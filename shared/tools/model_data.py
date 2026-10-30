"""Cuts chaos-firmware runs into fixed-length windows for model fitting.

Each window holds, per 4 ms tick: the goal or duty the servo was given,
the PWM it applied, the supply and temperature it reported, whether
torque was off, and the measured position. It starts from the measured
angle and velocity. Windows are tagged with their kind (trajectory, drop,
motor, chaos), run, setup, kp, supply and temperature, so fits and
validation sets can be chosen by tag.
"""

import numpy as np
import pandas as pd

import servolab

TICK_S = 0.004
TRAJECTORY_BLOCKS = {'chirp', 'chirp_soft', 'dual_sine', 'dual_sine_soft', 'raise_lower'}
MOTOR_BLOCKS = {'laps', 'heavy_laps', 'probe'}

# Healthy recordings and the setup each used. Runs after these (the hot
# runs of 2026-10-30 from 14:25 on) come from a unit that was failing.
RUNS = {
    '173504_unloaded': 'unloaded',
    '181040_arm-only-laps': 'arm',
    '182307_arm-13.5g': 'arm',
    '190523_batch-100.4g': '100g',
    '192200_trajectories-100.4g': '100g',
    '004203_heavy-first-turns': '1.52kg',
    '093448_heavy-batch': '1.52kg',
    '101658_heavy-batch-probed': '1.52kg-50mm',
    '105115_heavy-batch-probed': '1.52kg-50mm',
    '112510_heavy-batch-probed': '1.52kg-50mm',
    '115756_heavy-batch-probed': '1.52kg-50mm',
    '134414_heavy-batch-burn': '1.52kg-50mm',
}
# Last trusted time (s) where a run goes on past what is usable.
RUN_END_S = {'134414_heavy-batch-burn': 917.0}  # the burn: full-duty heating
# Blocks to skip: this batch wrote each goal just before the read, which
# lost most reads, so its logged goals don't match what the servo had.
SKIP_BLOCKS = {'190523_batch-100.4g': TRAJECTORY_BLOCKS}
# Firmware v1 wrote the duty before switching into motor mode, which clears
# it: motor segments that started with a mode switch applied 0.
V1_MODE_ENTRY_BUG = {'173504_unloaded'}


def kind_of(block_name, mode):
    if block_name in TRAJECTORY_BLOCKS and mode == servolab.POSITION:
        return 'trajectory'
    if block_name == 'lift_drop' and mode == servolab.RELEASED:
        return 'drop'
    if block_name in MOTOR_BLOCKS and mode == servolab.MOTOR:
        return 'motor'
    if block_name == 'chaos':
        return 'chaos'
    return None


def windows(key, setup, length_s=6.0, min_s=2.0):
    """Windows of one run, as a list of dicts of numpy arrays plus tags."""
    df = servolab.load('sc15', key)
    names = df.attrs['blocks']
    if 'block' not in df:  # v1 frames: chaos only, factory gain
        df['block'], df['kp'], names = 0, 15, {0: 'chaos'}
    if key in V1_MODE_ENTRY_BUG:
        first = df.groupby('segment')['flags'].transform('first')
        df.loc[(df['mode'] == servolab.MOTOR) & (first & servolab.FLAG_MODE_ENTRY != 0), 'command'] = 0
    end = RUN_END_S.get(key)
    if end is not None:
        df = df[df.t_s < end]
    df = df.reset_index(drop=True)
    # Read failures hold the last good values; position stays NaN for scoring.
    held = df[['position', 'speed', 'load', 'voltage_v', 'temperature_c']].ffill()
    temp = held.temperature_c.rolling(25, center=True, min_periods=1).median()
    n = int(length_s / TICK_S)
    out = []
    skip = SKIP_BLOCKS.get(key, set())
    tags = [None if names.get(b) in skip else kind_of(names.get(b), m) for b, m in zip(df.block, df['mode'])]
    df['kind'] = tags
    # Contiguous stretches of one kind; chaos keeps all its segments together.
    stretch = (df.kind != df.kind.shift()) | (df.block != df.block.shift())
    for _, g in df.groupby(stretch.cumsum()):
        kind = g.kind.iloc[0]
        if kind is None:
            continue
        if kind != 'chaos':
            # Trajectory, drop and motor stretches: one segment each.
            parts = [s for _, s in g.groupby('segment')]
        else:
            parts = [g]
        for part in parts:
            for start in range(0, len(part) - int(min_s / TICK_S) + 1, n):
                w = part.iloc[start:start + n]
                i0 = w.index[0]
                if not (df.valid[i0] and np.isfinite(df.vel[i0])):
                    continue
                pos = df.position[w.index].to_numpy(float)
                valid = df.valid[w.index].to_numpy(bool)
                if valid.mean() < 0.5:
                    continue
                out.append(dict(
                    run=key, setup=setup, kind=kind, block=names.get(int(w.block.iloc[0])),
                    kp=int(w.kp.iloc[0]), t0=float(w.t_s.iloc[0]),
                    supply=float(held.voltage_v[w.index].median()), temp=float(temp[w.index].median()),
                    mode=w['mode'].to_numpy(np.int64),
                    command=w.command.to_numpy(float), command_time=w.command_time_ms.to_numpy(float),
                    flags=w['flags'].to_numpy(np.int64),
                    load=held.load[w.index].to_numpy(float), volt=held.voltage_v[w.index].to_numpy(float),
                    tempc=temp[w.index].to_numpy(float), pos=pos, valid=valid,
                    vel=df.vel[w.index].to_numpy(float),
                    q0=float(df.deg[i0]), dq0=float(df.vel[i0]), u0=float(held.load[i0]),
                ))
    return out


def all_windows(keys=None, **kw):
    rows = []
    for key, setup in RUNS.items():
        if keys is None or key in keys:
            rows += windows(key, setup, **kw)
    return rows


if __name__ == '__main__':
    ws = all_windows()
    t = pd.DataFrame([{k: w[k] for k in ('run', 'setup', 'kind', 'block', 'kp')} for w in ws])
    print(len(ws), 'windows')
    print(t.groupby(['setup', 'kind']).size().unstack(fill_value=0))
