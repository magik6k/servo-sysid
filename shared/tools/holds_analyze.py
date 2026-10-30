#!/usr/bin/env python3
"""Fit gravity, gear efficiency and torque per PWM unit from a HOLDS run.

The chaos firmware's HOLDS program steps the pendulum through a series of
angles in position mode, sweeping up then down. At rest the SC15 gearbox
carries the load by friction alone (PWM output 0 or just the P term), so
the fit uses the middle of each step move instead, where the shaft turns
slowly (~16 deg/s).

The gearbox is asymmetric: lifting, the motor supplies gravity / forward
efficiency; lowering, gravity back-drives and reaches the motor scaled by
the back efficiency. So with m = PWM pushing along the motion and
g = gravity torque opposing the motion:

    m = base + a * max(g, 0) + b * min(g, 0)

base is back-EMF at the move speed plus Coulomb friction, 1/a the output
torque per PWM unit when lifting, b/a the forward x back efficiency. The
true down is found by a grid search over the gravity zero.
"""

import argparse

import numpy as np
import pandas as pd

POSITION = 1
G = 9.81


def per_segment(df, move_window):
    """Settled hold and mid-move means for each position-mode segment."""
    rows = []
    prev_goal = None
    for segment, g in df[df['mode'] == POSITION].groupby('segment'):
        t = g.t_s - g.t_s.iloc[0]
        goal = g.command.iloc[0]
        rest = g[t >= t.iloc[-1] - 1.0]
        move = g[(t >= move_window[0]) & (t <= move_window[1])]
        rows.append(dict(segment=segment, goal=goal,
                         step=np.nan if prev_goal is None else goal - prev_goal,
                         rest_position=rest.position.mean(), rest_load=rest.load.mean(),
                         move_position=move.position.mean(), move_load=move.load.mean()))
        prev_goal = goal
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv')
    ap.add_argument('--mass', type=float, required=True, help='Added mass, kg')
    ap.add_argument('--radius', type=float, required=True, help='Added mass radius, m')
    ap.add_argument('--arm-mass', type=float, default=0.0135)
    ap.add_argument('--arm-com', type=float, default=0.0615)
    ap.add_argument('--down', type=float, default=511, help='Nominal down count')
    ap.add_argument('--counts-per-rev', type=float, default=1610)
    ap.add_argument('--step', type=float, default=45, help='Regular step size in counts')
    args = ap.parse_args()

    df = pd.read_csv(args.csv).dropna(subset=['position'])
    seg = per_segment(df, (0.15, 0.50))
    rad_per_count = 2 * np.pi / args.counts_per_rev

    rest_nonzero = (seg.rest_load.abs() > 1).mean()
    print(f'{args.csv}: {len(seg)} holds; at rest {rest_nonzero * 100:.0f}% need any PWM '
          f'(max |u| {seg.rest_load.abs().max():.0f})')

    moves = seg[(seg.step.abs() - args.step).abs() <= 5].copy()
    moves['dir'] = np.sign(moves.step)
    moment = args.mass * args.radius + args.arm_mass * args.arm_com
    tau_amp = G * moment

    # Positive PWM drives the count down, so the push along a count-up
    # motion is -u.
    push = -moves.move_load * moves.dir
    best = None
    for zero_deg in np.arange(-20, 20.25, 0.25):
        theta = (moves.move_position - args.down) * rad_per_count - np.radians(zero_deg)
        opposing = tau_amp * np.sin(theta) * moves.dir
        x = np.column_stack([np.ones(len(moves)), np.clip(opposing, 0, None), np.clip(opposing, None, 0)])
        coef, *_ = np.linalg.lstsq(x, push, rcond=None)
        rms = np.sqrt(np.mean((push - x @ coef) ** 2))
        if best is None or rms < best[0]:
            best = (rms, zero_deg, coef)
    rms, zero_deg, (base, a, b) = best

    print(f'Moves fitted: {len(moves)}, residual RMS {rms:.1f} PWM units')
    print(f'Gravity: max {tau_amp:.4f} N*m ({moment * 1000:.2f} g*m)')
    print(f'  -> true down = nominal {zero_deg:+.2f} deg = count {args.down + np.radians(zero_deg) / rad_per_count:.0f}')
    print(f'  -> lifting: {1000 / a:.2f} mN*m at the output per PWM unit '
          f'(linear extrapolation to 1000 units: {1000 / a:.2f} N*m)')
    print(f'  -> lowering reaches the motor at {b / a:.2f} of the lifting rate '
          f'(forward x back efficiency; ~{np.sqrt(max(b / a, 0)):.2f} each if equal)')
    print(f'  -> base push along the motion {base:.1f} PWM units (back-EMF at the move speed + Coulomb friction)')


if __name__ == '__main__':
    main()
