#!/usr/bin/env python3
"""Step the hold_position sketch through a list of goals and log telemetry.

Resets the board (it boots into a 2 s move to its GOAL and holds), then
sends each goal and waits --dwell seconds. Writes
<repo>/<servo>/data/holdsteps_<stamp>_<tag>.csv.gz and prints a per-step
summary. Board firmware: shared/firmware/hold_position.
"""

import argparse
import csv
import datetime
import gzip
import pathlib
import re
import time

import serial

import servolab

LINE = re.compile(r't (\d+)\s+pos\s+(\d+)\s+load ([+-])(\d+)\s+volt ([\d.]+)\s+temp (\d+)\s+torque (\d)')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('goals', help='comma-separated counts, e.g. 556,466,511')
    ap.add_argument('--dwell', type=float, default=3.0)
    ap.add_argument('--tag', default='steps')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--servo', default='sc15')
    args = ap.parse_args()
    goals = [int(g) for g in args.goals.split(',')]

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.port, 115200, 0.05
    s.dtr = s.rts = False
    s.open()
    s.rts = True
    time.sleep(0.1)
    s.rts = False

    rows, pending, goal = [], b'', None
    schedule = [(3.5 + i * args.dwell, g) for i, g in enumerate(goals)]
    t0 = time.time()
    end = schedule[-1][0] + args.dwell
    while time.time() - t0 < end:
        now = time.time() - t0
        if schedule and now >= schedule[0][0]:
            goal = schedule.pop(0)[1]
            s.write(f'{goal}\n'.encode())
        pending += s.read(512)
        *lines, pending = pending.split(b'\n')
        for raw in lines:
            m = LINE.search(raw.decode(errors='replace'))
            if not m:
                continue
            t, pos, sign, load, volt, temp, torque = m.groups()
            rows.append(dict(t_s=int(t) / 1000, goal=goal, position=int(pos),
                             load=int(load) * (-1 if sign == '-' else 1),
                             voltage_v=float(volt), temperature_c=int(temp),
                             torque=int(torque)))

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = servolab.data_dir(args.servo) / f'holdsteps_{stamp}_{args.tag}.csv.gz'
    with gzip.open(out, 'wt', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f'{len(rows)} rows -> {out}')

    print(f'{"goal":>5} {"final":>5} {"err":>4} {"peak load":>9} {"hold load":>9} '
          f'{"min V":>5} {"temp":>4}')
    for g in goals:
        part = [r for r in rows if r['goal'] == g]
        if not part:
            continue
        settled = [r for r in part if r['t_s'] >= part[0]['t_s'] + 2.3]
        final = part[-1]['position']
        hold = max((abs(r['load']) for r in settled), default=0)
        print(f'{g:5d} {final:5d} {final - g:+4d} {max(abs(r["load"]) for r in part):9d} '
              f'{hold:9d} {min(r["voltage_v"] for r in part):5.1f} '
              f'{max(r["temperature_c"] for r in part):4d}')


if __name__ == '__main__':
    main()
