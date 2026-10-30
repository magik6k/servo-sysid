#!/usr/bin/env python3
"""Fit actuator parameters from a chaos-test CSV (tools/chaos_log.py).

Prints the numbers the simulator's actuator model needs: output slew
rates, ramp reset on mode entry, speed map and time constant in motor
mode, position-mode step response, sensor span and supply behaviour.
Velocities are in position counts/s; PWM in units of 0.1% duty.
"""

import argparse

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

MOTOR, POSITION = 0, 1
FLAG_MODE_ENTRY = 0x02
SAMPLE_S = 0.004
VALID_LO, VALID_HI = 8, 1014  # Readings outside are pinned in the blind spot.

# Readings to trust for derivatives: the linear part of the window (values
# creep near the pins), excluding the 1-3 sample sweep across the whole
# range when the wiper crosses the track gap in the blind spot. Nothing
# real moves more than ~10 counts per 4 ms sample.
TRUST_LO, TRUST_HI = 20, 1000
MAX_STEP = 30


def add_derivatives(df):
    """Velocity and acceleration from position, within trusted runs only."""
    step = df.position.diff().abs()
    df['valid'] = (df.position.between(TRUST_LO, TRUST_HI)
                   & (step <= MAX_STEP) & (step.shift(-1) <= MAX_STEP))
    df['vel'] = np.nan
    df['acc'] = np.nan
    runs = (~df.valid).cumsum()

    for _, g in df[df.valid].groupby(runs[df.valid]):
        if len(g) < 17:
            continue
        pos = g.position.to_numpy(float)
        vel = savgol_filter(pos, 9, 2, deriv=1, delta=SAMPLE_S)
        acc = savgol_filter(pos, 15, 2, deriv=2, delta=SAMPLE_S)
        df.loc[g.index[4:-4], 'vel'] = vel[4:-4]
        df.loc[g.index[7:-7], 'acc'] = acc[7:-7]


def output_rates(df, mode):
    g = df[df['mode'] == mode]
    step = (g.segment == g.segment.shift()) & (g.t_s.diff() < 1.5 * SAMPLE_S)
    du = g.load.diff()[step]
    rising = du[g.load.abs()[step] > g.load.shift().abs()[step]].abs()
    falling = du[g.load.abs()[step] < g.load.shift().abs()[step]].abs()
    per_ms = SAMPLE_S * 1e3
    return rising / per_ms, falling / per_ms


def ramp_reset(df):
    """|PWM output| on the frame after a motor-mode entry that interrupted a strong output."""
    after = []
    entries = (df['flags'] & FLAG_MODE_ENTRY > 0) & (df['mode'] == MOTOR)
    for i in df.index[entries]:
        if i - 1 in df.index and i + 1 in df.index and abs(df.load[i - 1]) > 200:
            after.append(abs(df.load[i + 1]))
    return np.array(after)


def steady_state(df):
    """Velocity once the output has equalled the command for over 300 ms."""
    held_same = ((df.load == df.load.shift()) & (df.segment == df.segment.shift())
                 & (df['mode'] == MOTOR))
    held_for = df.groupby((~held_same).cumsum()).t_s.transform(lambda t: t - t.iloc[0])
    ss = df[(df['mode'] == MOTOR) & (held_for > 0.3) & df.vel.notna()
            & (df.load == df.command)]
    table = ss.groupby((ss.load / 50).round() * 50).vel.agg(['median', 'count'])
    return table[table['count'] > 15]


def first_order_fit(df):
    """acc = a*u - b*vel - c*sign(vel) over motor-mode samples."""
    f = df[(df['mode'] == MOTOR) & df.acc.notna() & (df.vel.abs() > 100)]
    x = np.column_stack([f.load, -f.vel, -np.sign(f.vel)])
    coef, *_ = np.linalg.lstsq(x, f.acc, rcond=None)
    r2 = 1 - np.var(f.acc - x @ coef) / np.var(f.acc)
    return coef, r2, len(f)


def position_steps(df):
    """Flat-out (goal time 1 ms) position-mode steps of at least 100 counts."""
    rows = []
    for _, g in df[df['mode'] == POSITION].groupby('segment'):
        goal, start = g.command.iloc[0], g.position.iloc[0]
        if g.command_time_ms.iloc[0] != 1 or abs(goal - start) < 100:
            continue
        err = (g.position - goal).to_numpy()
        inside = np.flatnonzero(np.abs(err) <= 10)
        if not len(inside):
            continue
        t_ms = (g.t_s.iloc[inside[0]] - g.t_s.iloc[0]) * 1e3
        overshoot = max(0, (np.sign(goal - start) * (g.position - goal)).max())
        rows.append((abs(goal - start), t_ms, overshoot, abs(err[-1])))
    return pd.DataFrame(rows, columns=['step', 'settle_ms', 'overshoot', 'final_err'])


def laps(df):
    """Same-direction mid-range crossings at near-constant output: one lap each."""
    rows = []
    for _, g in df[df['mode'] == MOTOR].groupby('segment'):
        pos, t = g.position.to_numpy(), g.t_s.to_numpy()
        load, speed = g.load.to_numpy(), g.speed.to_numpy()
        for sign in (1, -1):
            cross = np.flatnonzero((sign * (pos[:-1] - 512) < 0) & (sign * (pos[1:] - 512) >= 0))
            for a, b in zip(cross[:-1], cross[1:]):
                w = slice(a, b + 1)
                if np.ptp(load[w]) > 30 or np.abs(load[w]).min() < 100:
                    continue
                valid = (pos[w] >= VALID_LO) & (pos[w] <= VALID_HI)
                rows.append((t[b] - t[a], abs(np.median(speed[w][valid])), valid.mean()))
    r = pd.DataFrame(rows, columns=['period_s', 'speed', 'valid_fraction'])
    r['counts_per_rev'] = r.period_s * r.speed
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv')
    args = ap.parse_args()

    raw = pd.read_csv(args.csv)
    df = raw.dropna(subset=['position']).copy()
    add_derivatives(df)

    period_ms = np.diff(raw.t_s) * 1e3
    print(f'{args.csv}')
    print(f'  {len(raw)} frames, {raw.t_s.iloc[-1]:.0f} s, {raw.segment.max()} segments, '
          f'{100 * (1 - len(df) / len(raw)):.2f}% failed reads, '
          f'period p99 {np.percentile(period_ms, 99):.2f} ms')

    print('\nPWM output slew (units/ms, median / p95):')
    for name, mode in (('motor', MOTOR), ('position', POSITION)):
        up, down = output_rates(df, mode)
        print(f'  {name:8s} rising {up.median():.2f} / {up.quantile(.95):.2f}   '
              f'falling {down.median():.2f} / {down.quantile(.95):.2f}')
    reset = ramp_reset(df)
    if len(reset):
        print(f'  motor-mode entry from |u|>200: next-frame |u| median {np.median(reset):.0f} '
              f'(max {reset.max():.0f}, n={len(reset)})')

    v = df[(df['mode'] == MOTOR) & df.vel.notna() & (df.speed.abs() < 5000)]
    k = np.polyfit(v.vel, v.speed, 1)
    print(f'\nSpeed register = {k[0]:.3f} x dpos/dt {k[1]:+.0f} '
          f'(r={np.corrcoef(v.vel, v.speed)[0, 1]:.3f})')

    table = steady_state(df)
    print('\nMotor-mode steady state (velocity by PWM output):')
    print(table['median'].round(0).to_frame('vel').T.to_string())
    for sign in (1, -1):
        t = table[(np.sign(table.index) == sign) & (table['median'].abs() > 100)]
        if len(t) > 2:
            gain, offset = np.polyfit(np.abs(t.index), np.abs(t['median']), 1)
            print(f'  PWM {"+" if sign > 0 else "-"}: |vel| = {gain:.3f} x (|u| - {-offset / gain:.0f}),'
                  f' full {gain * 1000 + offset:.0f} counts/s, velocity sign {-sign:+d}')

    (a, b, c), r2, n = first_order_fit(df)
    print(f'\nMotor-mode dynamics: acc = {a:.1f} u - {b:.2f} vel - {c:.0f} sign(vel)'
          f'  -> tau {1000 / b:.0f} ms, gain {a / b:.2f} counts/s per unit (R2 {r2:.2f}, n={n})')

    steps = position_steps(df)
    if len(steps):
        print(f'\nPosition mode, flat-out steps (n={len(steps)}): within 10 counts after '
              f'{steps.settle_ms.median():.0f} ms median for a {steps.step.median():.0f}-count step; '
              f'overshoot {steps.overshoot.median():.0f}, final error {steps.final_err.median():.0f} counts')
        fit = np.polyfit(steps.step, steps.settle_ms, 1)
        print(f'  settle ms ~= {fit[1]:.0f} + {fit[0]:.3f} x step')

    lap = laps(df)
    if len(lap):
        cpr = lap.counts_per_rev.median()
        print(f'\nSensor (from {len(lap)} laps): {cpr:.0f} counts/rev -> 0..1023 spans '
              f'{1023 / cpr * 360:.0f} deg; valid {lap.valid_fraction.median() * 100:.0f}% of a turn')

    err = df[df.error > 0]
    print(f'\nSupply {df.voltage_v.min():.1f}..{df.voltage_v.max():.1f} V; '
          f'error bits {sorted(int(e) for e in df.error.unique())} in {len(err)} frames; '
          f'temperature {df.temperature_c.min():.0f}..{df.temperature_c.max():.0f} C')


if __name__ == '__main__':
    main()
