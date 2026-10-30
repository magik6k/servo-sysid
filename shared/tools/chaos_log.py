#!/usr/bin/env python3
"""Record chaos-test telemetry from the Atom Lite into a CSV file.

Resets the board, keeps the '#' preamble (seed, plan, register dump) as
a .meta.txt next to the CSV, then decodes the binary frames sent by
shared/firmware/chaos/chaos.ino until --seconds have passed, or until
Ctrl-C or SIGTERM, which still keep the file. The CSV is gzipped at the
end, into <repo>/<servo>/data unless --out says otherwise. The
preamble's frame_bytes picks the layout: 24 (v1) or 26 (v2, adds block and kp).
"""

import argparse
import csv
import datetime
import gzip
import pathlib
import re
import signal
import struct
import sys
import time

import serial

import servolab

# Must match struct Frame in shared/firmware/chaos/chaos.ino, by frame_bytes.
FRAMES = {
    24: struct.Struct('<2sIHBBhHHhhBBBB'),
    26: struct.Struct('<2sIHBBhHHhhBBBBBB'),
}
SYNC = b'\xA5\x5A'

FLAG_READ_FAILED = 0x04
MODE_FAULT = 3  # Stall guard tripped: torque off, segment ended.

COLUMNS = [
    't_s', 'segment', 'mode', 'flags', 'command', 'command_time_ms',
    'position', 'speed', 'load', 'voltage_v', 'temperature_c', 'error',
]
V2_COLUMNS = ['block', 'kp']


def frame_bytes(meta):
    for line in meta:
        m = re.search(r'frame_bytes=(\d+)', line)
        if m:
            return int(m.group(1))
    sys.exit('Preamble has no frame_bytes')


def open_port(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.1
    s.dtr = False
    s.rts = False
    s.open()
    return s


def open_board(port):
    s = open_port(port)

    # Pulse EN through RTS so recording starts from a fresh boot.
    s.rts = True
    time.sleep(0.1)
    s.rts = False
    return s


def reopen(port, deadline):
    """Reopens after a USB dropout without resetting the board.

    Servo current transients can make the Atom's USB serial adapter
    re-enumerate. The ESP32 keeps running its plan, so recording resumes
    in the same boot and t_s stays continuous. Returns None at deadline.
    """
    while time.time() < deadline:
        time.sleep(0.2)
        try:
            return open_port(port)
        except serial.SerialException:
            pass
    return None


def read_preamble(s, timeout_s=10):
    """Returns ('#' lines, leftover binary bytes after '# BEGIN')."""
    meta = []
    buf = b''
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        buf += s.read(256)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode(errors='replace').strip()
            if not text.startswith('#'):
                continue  # Boot ROM chatter.
            meta.append(text)
            if text == '# BEGIN':
                return meta, buf

    sys.exit(f'No "# BEGIN" from the board within {timeout_s} s')


def stop(signum, frame):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, stop)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--seconds', type=float, default=120)
    ap.add_argument('--servo', default='sc15', help='Servo directory in the repo')
    ap.add_argument('--out', type=pathlib.Path, help='Default: <repo>/<servo>/data')
    ap.add_argument('--tag', default='unloaded',
                    help='Short label for the setup, e.g. unloaded, disk-50g')
    args = ap.parse_args()

    args.out = args.out or servolab.data_dir(args.servo)
    args.out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = args.out / f'chaos_{stamp}_{args.tag}.csv'
    meta_path = csv_path.with_suffix('.meta.txt')

    s = open_board(args.port)
    meta, buf = read_preamble(s)
    meta_path.write_text('\n'.join(meta + [f'# tag {args.tag}']) + '\n')
    size = frame_bytes(meta)
    if size not in FRAMES:
        sys.exit(f'Unknown frame size {size}')
    frame = FRAMES[size]
    extra_columns = V2_COLUMNS if size == 26 else []

    frames = bad = read_failures = 0
    stalls = []
    dropouts = []  # Board time of the last frame before each USB dropout.
    first_us = last_us = None
    wraps = 0
    max_temp = 0

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS + extra_columns)

        end = time.time() + args.seconds
        try:
            while time.time() < end:
                try:
                    buf += s.read(4096)
                except serial.SerialException:
                    s.close()
                    if last_us is not None:
                        dropouts.append(last_us + (wraps << 32))
                    s = reopen(args.port, end)
                    if s is None:
                        break
                    buf = b''  # Drop any partial frame from before the gap.
                    continue

                while True:
                    i = buf.find(SYNC)
                    if i < 0:
                        buf = buf[-1:]
                        break
                    if len(buf) - i < frame.size:
                        buf = buf[i:]
                        break

                    raw = buf[i:i + frame.size]
                    if sum(raw[2:-1]) & 0xFF != raw[-1]:
                        bad += 1
                        buf = buf[i + 1:]
                        continue
                    buf = buf[i + frame.size:]

                    fields = frame.unpack(raw)
                    (_, t_us, segment, mode, flags, command, command_time,
                     position, speed, load, voltage, temperature, error) = fields[:13]
                    extra = list(fields[13:-1])

                    if last_us is not None and t_us < last_us:
                        wraps += 1  # micros() wraps every ~71 minutes.
                    last_us = t_us
                    t_us += wraps << 32
                    if first_us is None:
                        first_us = t_us

                    frames += 1
                    if mode == MODE_FAULT:
                        stalls.append((t_us - first_us) / 1e6)
                    if flags & FLAG_READ_FAILED:
                        read_failures += 1
                        position = speed = load = voltage = temperature = error = ''
                    else:
                        max_temp = max(max_temp, temperature)
                        voltage = voltage / 10

                    writer.writerow([
                        f'{(t_us - first_us) / 1e6:.6f}', segment, mode, flags,
                        command, command_time, position, speed, load,
                        voltage, temperature, error,
                    ] + extra)
        except KeyboardInterrupt:
            print('Stopped early', file=sys.stderr)

    if s is not None:
        s.close()
    gz_path = csv_path.with_name(csv_path.name + '.gz')
    with open(csv_path, 'rb') as src, gzip.open(gz_path, 'wb', compresslevel=9) as dst:
        dst.write(src.read())
    csv_path.unlink()

    span = (last_us + (wraps << 32) - first_us) / 1e6 if frames > 1 else 0
    print(f'{gz_path}')
    print(f'  {frames} frames over {span:.1f} s '
          f'({frames / span if span else 0:.1f} Hz), '
          f'{bad} bad checksums, {read_failures} failed servo reads, '
          f'max temperature {max_temp} C')
    if dropouts:
        print(f'  USB DROPPED {len(dropouts)}x, last frame before each at '
              + ', '.join(f'{(t - first_us) / 1e6:.2f}' for t in dropouts)
              + ' s; frames in each gap are lost')
    if stalls:
        print(f'  STALL GUARD TRIPPED {len(stalls)}x at '
              + ', '.join(f'{t:.2f}' for t in stalls) + ' s')


if __name__ == '__main__':
    main()
