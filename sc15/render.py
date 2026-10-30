#!/usr/bin/env python3
"""Render the SC15 characteristic charts into rendered/charts/.

Each chart is drawn twice, for light and dark page themes. rendered/README.md
references them by name; rerun this after adding data:

    python3 sc15/render.py
"""

import pathlib
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'shared' / 'tools'))
import servolab  # noqa: E402
from holds_analyze import per_segment  # noqa: E402

SERVO = 'sc15'
OUT = servolab.servo_dir(SERVO) / 'rendered' / 'charts'
CFG = servolab.config(SERVO)
CPD = CFG['counts_per_rev'] / 360

# Reference palette (dataviz skill): first three categorical slots, which
# validate all-pairs in both modes, plus text, grid and reference inks.
THEMES = {
    'light': dict(surface='#fcfcfb', text='#0b0b0b', text2='#52514e', grid='#e4e3df',
                  ref='#8a8984', series=['#2a78d6', '#eb6834', '#1baf7a']),
    'dark': dict(surface='#1a1a19', text='#f0efec', text2='#c3c2b7', grid='#383835',
                 ref='#8a8984', series=['#3987e5', '#d95926', '#199e70']),
}
LW = 1.0  # points; 2 px at the 150 dpi export

_runs = {}


def run(key):
    if key not in _runs:
        _runs[key] = servolab.load(SERVO, key)
    return _runs[key]


def style(th):
    plt.rcParams.update({
        'figure.facecolor': th['surface'], 'axes.facecolor': th['surface'],
        'savefig.facecolor': th['surface'], 'axes.edgecolor': th['grid'],
        'axes.labelcolor': th['text2'], 'xtick.color': th['text2'], 'ytick.color': th['text2'],
        'text.color': th['text'], 'axes.titlecolor': th['text'], 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'axes.labelsize': 9,
        'xtick.labelsize': 8, 'ytick.labelsize': 8, 'font.size': 9,
        'axes.grid': True, 'grid.color': th['grid'], 'grid.linewidth': 0.5, 'grid.linestyle': '-',
        'axes.spines.top': False, 'axes.spines.right': False,
        'legend.frameon': False, 'legend.fontsize': 8, 'legend.labelcolor': th['text2'],
        'lines.linewidth': LW, 'lines.solid_capstyle': 'round', 'lines.solid_joinstyle': 'round',
    })


def note(ax, th, x, y, text, **kw):
    ax.annotate(text, (x, y), color=th['text2'], fontsize=8, **kw)


# Each chart: (name, function(th) -> figure). Titles state the finding.

def motor_slew(th):
    df = run('181040_arm-only-laps')
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for color, duty in zip(th['series'], (400, 800, 1000)):
        seg = df[(df['mode'] == servolab.MOTOR) & (df.command == duty)].segment.iloc[0]
        g = df[df.segment == seg]
        t = (g.t_s - g.t_s.iloc[0]) * 1000
        keep = t <= 1100
        ax.plot(t[keep], g.load[keep].abs() / 10, color=color, label=f'command {duty // 10} %')
    t = np.array([0, 800])
    ax.plot(t, t * 1.25 / 10, color=th['ref'], lw=0.6)
    note(ax, th, 520, 72, '1.25 units/ms\n(5 per 4 ms tick)', ha='right')
    ax.set(xlabel='time since command (ms)', ylabel='applied output (%)', ylim=(0, 105),
           title='Motor mode slews the output at 1.25 units/ms: 0 → full takes 0.8 s')
    ax.legend(loc='lower right')
    return fig


def position_slew(th):
    df = run('bang-reversals')
    g = df[df.segment == 2]
    flips = g.index[g.command.diff().fillna(0) != 0]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for i in flips:
        sign = 1 if g.command[i] < g.command[i - 1] else -1  # +PWM drives the count down
        w = df.loc[i - 6:i + 45]
        t = (w.t_s - df.t_s[i]) * 1000
        ax.plot(t, sign * w.load / 10, color=th['series'][0], lw=LW)
    ax.axhline(0, color=th['ref'], lw=0.6)
    ax.axvline(0, color=th['ref'], lw=0.6)
    t = np.array([20, 180])
    ax.plot(t, (t - 20) * 5 / 10, color=th['ref'], lw=0.6)
    note(ax, th, 120, 15, 'builds at 5 units/ms')
    note(ax, th, 6, -85, 'collapses at\n~100 units/ms')
    ax.set(xlabel='time since the goal flipped (ms)', ylabel='output in the new direction (%)',
           title='Position mode drops its output fast on reversal, then builds it slowly')
    return fig


def turn_rates(df, segments):
    """Mean turn rate (RPM) per motor segment from same-direction bottom passes."""
    rows = []
    for seg in segments:
        g = df[df.segment == seg].dropna(subset=['position'])
        pos, t, cmd = g.position.to_numpy(), g.t_s.to_numpy(), int(g.command.iloc[0])
        sgn = -1 if cmd > 0 else 1
        ok = (pos > 100) & (pos < 924)
        down = CFG['down_count']
        passes = [t[i] for i in range(1, len(pos))
                  if ok[i] and ok[i - 1] and (pos[i - 1] - down) * sgn < 0 <= (pos[i] - down) * sgn]
        if len(passes) >= 2:
            rows.append((cmd, 60 / np.diff(passes).mean()))
    return pd.DataFrame(rows, columns=['duty', 'rpm'])


def speed_vs_pwm(th):
    df = run('181040_arm-only-laps')
    motor = df[(df['mode'] == servolab.MOTOR) & (df.command.abs() >= 100)]
    arm = turn_rates(df, motor.segment.unique())
    heavy = turn_rates(run('heavy-first-turns'), [15, 17, 19, 21, 23, 25])
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for color, (label, d, fmt) in zip(th['series'], [
            ('arm only, + direction', arm[arm.duty > 0], 'o-'),
            ('arm only, − direction', arm[arm.duty < 0], 'o-'),
            ('1.52 kg at 65 mm, each direction', heavy, 'o')]):
        ax.plot(d.duty.abs() / 10, d.rpm, fmt, color=color, ms=4.5, mec=th['surface'], mew=1, label=label)
    ax.set(xlabel='PWM command (%)', ylabel='mean turn rate (RPM)', xlim=(0, 105), ylim=(0, 85),
           title='Speed is linear in PWM: ~75 RPM at full, ~2.1 counts/s per unit')
    ax.legend(loc='upper left')
    return fig


def sensor_window(th):
    df = run('181040_arm-only-laps')
    seg = df[(df['mode'] == servolab.MOTOR) & (df.command == 400)].segment.iloc[0]
    g = df[df.segment == seg]
    g = g[(g.t_s - g.t_s.iloc[0] > 2) & (g.t_s - g.t_s.iloc[0] < 6.5)]
    t = g.t_s - g.t_s.iloc[0]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(t, g.position, color=th['series'][0], lw=0.8)
    for y in CFG['trust_counts']:
        ax.axhline(y, color=th['ref'], lw=0.6)
    note(ax, th, t.iloc[0], 1003, 'trusted window 20..1000', ha='left', va='bottom')
    ax.set(xlabel='time (s), constant 40 % PWM', ylabel='position register',
           title='0..1023 covers ~227°; the other ~133° reads pinned at 0 or 1022')
    return fig


def speed_register(th):
    df = run('173504_unloaded')
    d = df[df.valid & df.vel.notna() & (df.speed.abs() < 3000)].iloc[::3]
    fig, ax = plt.subplots(figsize=(5.5, 4.4))
    ax.scatter(d.vel * CPD, d.speed, s=2, color=th['series'][0], alpha=0.25, linewidths=0)
    lim = np.array([-2500, 2500])
    ax.plot(lim, lim, color=th['ref'], lw=0.6)
    ax.set(xlabel='d(position)/dt (counts/s)', ylabel='speed register (counts/s)',
           xlim=lim, ylim=lim, title='Speed register = counts/s, quantized to 50')
    return fig


def step_response(th):
    df = run('173504_unloaded').dropna(subset=['position'])
    rows = []
    for _, g in df[df['mode'] == servolab.POSITION].groupby('segment'):
        goal, start = g.command.iloc[0], g.position.iloc[0]
        if g.command_time_ms.iloc[0] != 1 or abs(goal - start) < 100:
            continue
        inside = np.flatnonzero(np.abs(g.position - goal) <= 10)
        if len(inside):
            rows.append((abs(goal - start), (g.t_s.iloc[inside[0]] - g.t_s.iloc[0]) * 1000))
    r = pd.DataFrame(rows, columns=['step', 'ms'])
    slope, icept = np.polyfit(r.step, r.ms, 1)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.scatter(r.step / CPD, r.ms, s=14, color=th['series'][0], edgecolors=th['surface'], linewidths=1)
    x = np.array([r.step.min(), r.step.max()])
    ax.plot(x / CPD, icept + slope * x, color=th['ref'], lw=0.8)
    note(ax, th, x[1] / CPD, icept + slope * x[1], f'  {icept:.0f} ms + {slope:.2f} ms/count', va='center')
    ax.set(xlabel='step size (deg)', ylabel='time to within 10 counts (ms)',
           title='Flat-out position steps, unloaded: no overshoot, ±1 count final error')
    return fig


def torque_per_pwm(th):
    # Holds and 100 g laps numbers come from shared/tools/holds_analyze.py and
    # the quasi-static laps regression (see sc15/README.md); the heavy stall
    # points are recomputed here from the stall angles.
    heavy = run('heavy-first-turns')
    tg = servolab.gravity_torque_max(CFG, '1.52kg')
    stalls = []
    for seg in (11, 13):
        g = heavy[heavy.segment == seg]
        deg = g.deg.iloc[-1]
        stalls.append(tg * abs(np.sin(np.radians(deg))) / abs(g.command.iloc[0]) * 1000)
    points = [
        ('100 g holds, cold', 6, 3, 3.26),
        ('100 g holds, warm', 6, 3, 3.58),
        ('100 g laps', 20, 5, 2.6),
        ('1.52 kg stall at 35 %', 35, 0, float(np.mean(stalls))),
    ]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for label, x, xr, y in points:
        ax.errorbar(x, y, xerr=xr or None, fmt='o', color=th['series'][0], ms=5,
                    mec=th['surface'], mew=1, elinewidth=LW, capsize=0)
        note(ax, th, x + xr, y, f'  {label}: {y:.2f}', va='center')
    ax.errorbar(35, np.mean(stalls), yerr=[[np.mean(stalls) - min(stalls)], [max(stalls) - np.mean(stalls)]],
                fmt='none', color=th['series'][0], elinewidth=LW, capsize=0)
    ax.set(xlabel='output level (%)', ylabel='lifting torque per 0.1 % (mN·m)', xlim=(0, 60), ylim=(0, 4.2),
           title='Torque per unit falls with output: not linear to full power')
    return fig


def gearbox_asymmetry(th):
    df = run('184653_holds').dropna(subset=['position'])
    seg = per_segment(df, (0.15, 0.50))
    moves = seg[(seg.step.abs() - 45).abs() <= 5].copy()
    moves['dir'] = np.sign(moves.step)
    tg = servolab.gravity_torque_max(CFG, '100g')
    rad = np.radians((moves.move_position - CFG['down_count']) / CPD)
    moves['opposing'] = tg * np.sin(rad) * moves.dir
    moves['push'] = -moves.move_load * moves.dir
    x = np.column_stack([np.ones(len(moves)), moves.opposing.clip(lower=0), moves.opposing.clip(upper=0)])
    (base, a, b), *_ = np.linalg.lstsq(x, moves.push, rcond=None)
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for color, (label, d) in zip(th['series'], [('moving toward + angles', moves[moves.dir > 0]),
                                                ('moving toward − angles', moves[moves.dir < 0])]):
        ax.scatter(d.opposing * 1000, d.push, s=16, color=color, edgecolors=th['surface'], linewidths=1, label=label)
    g = np.linspace(moves.opposing.min(), moves.opposing.max(), 50)
    ax.plot(g * 1000, base + a * g.clip(0) + b * g.clip(None, 0), color=th['ref'], lw=0.8)
    note(ax, th, 5, 79, f'lifting: {1000 / a:.2f} mN·m per unit', va='bottom')
    note(ax, th, -135, 47, f'lowering: {b / a:.2f}× that slope\n(efficiency ≈ {np.sqrt(b / a):.2f} each way)', va='bottom')
    ax.axvline(0, color=th['ref'], lw=0.6)
    ax.set(xlabel='gravity torque opposing the motion (mN·m)', ylabel='PWM pushing along the motion (units)',
           title='Gearbox is lossy both ways: lowering needs push, not braking (100 g, 16°/s)')
    ax.legend(loc='upper left')
    return fig


def heavy_speed_vs_angle(th):
    df = run('heavy-first-turns')
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for color, seg in zip(th['series'], (15, 19, 23)):
        g = df[(df.segment == seg) & df.vel.notna()]
        g = g[g.t_s - g.t_s.iloc[0] > 0.8]
        along = -np.sign(g.command.iloc[0])  # +PWM drives the angle down
        ax.scatter(g.deg * along, (g.vel * along), s=2, color=color, linewidths=0, alpha=0.6,
                   label=f'{abs(g.command.iloc[0]) // 10} % PWM')
    ax.axvline(0, color=th['ref'], lw=0.6)
    note(ax, th, 35, 60, 'rising side:\ngravity opposes', ha='center')
    note(ax, th, -55, 150, 'falling side:\ngravity overruns the motor', ha='center')
    ax.set(xlabel='angle along the direction of travel (deg, 0 = hanging down)', ylabel='speed (deg/s)',
           ylim=(0, 720), title='1.52 kg at 65 mm (0.98 N·m): crawls past horizontal, falls at ~500°/s')
    leg = ax.legend(loc='upper right', markerscale=5)
    for h in leg.legend_handles:
        h.set_alpha(1)
    return fig


def regen_clamp(th):
    before = run('heavy-first-turns')
    after = run('regen-soak')
    fig, axes = plt.subplots(2, 1, figsize=(7, 4.4), sharex=True, sharey=True)
    for ax, (label, g) in zip(axes, [('no clamp', before[before.segment == 23]),
                                     ('diode clamp', after[after.segment == 2])]):
        t = g.t_s - g.t_s.iloc[0]
        keep = t <= 4.0
        ax.plot(t[keep], g.voltage_v[keep], color=th['series'][0], lw=0.8)
        ax.axhline(9.9, color=th['ref'], lw=0.6)
        share = (g.voltage_v[keep] >= 9.9).mean() * 100
        ax.set_title(f'{label}: {share:.0f} % of samples at the 9.9 V reading ceiling', fontsize=9,
                     fontweight='normal', color=th['text2'])
        ax.set_ylabel('supply (V)')
    axes[1].set_xlabel('time in an 80 % lap burst (s)')
    fig.suptitle('Regen in 80 % laps: the clamp halves the time at the 9.9 V reading ceiling',
                 x=0.02, ha='left', fontsize=10, fontweight='bold', color=th['text'])
    fig.tight_layout()
    return fig


def bang(th):
    df = run('bang-reversals')
    g = df[df.segment.between(1, 3)]
    t0 = df[df.segment == 2].t_s.iloc[0]
    g = g[(g.t_s >= t0 - 0.3) & (g.t_s <= t0 + 2.3)]
    t = g.t_s - t0
    fig, axes = plt.subplots(3, 1, figsize=(7, 5.6), sharex=True)
    for ax, (col, scale, label) in zip(axes, [('deg', 1, 'angle (deg)'), ('load', -0.1, 'output (%)'),
                                              ('voltage_v', 1, 'supply (V)')]):
        ax.plot(t, g[col] * scale, color=th['series'][0], lw=0.8)
        ax.set_ylabel(label)
    axes[2].set_xlabel('time (s)')
    axes[0].set_title('Full-speed reversals near horizontal (1.52 kg): supply sags to 6.9 V')
    fig.tight_layout()
    return fig


def static_hold(th):
    light = run('batch-100.4g')
    fig, ax = plt.subplots(figsize=(7, 3.8))
    released = light[light['mode'] == servolab.RELEASED]
    for i, (_, g) in enumerate(released.groupby('segment')):
        t = g.t_s - g.t_s.iloc[0]
        ax.plot(t, g.deg, color=th['series'][0], lw=LW, label='100 g at 125 mm, torque off' if i == 0 else None)
        note(ax, th, t.iloc[-1], g.deg.median(), f' {g.deg.median():+.0f}°', va='center')
    heavy = run('heavy-first-turns')
    g = heavy[(heavy.segment == 26) & heavy.position.between(40, 1013)]  # slow creep: trust past 1000
    t = g.t_s - g.t_s.iloc[0]
    ax.plot(t, g.deg, color=th['series'][1], lw=LW, label='1.52 kg at 65 mm, 0 % PWM (motor mode)')
    ax.set(xlabel='time since released (s)', ylabel='angle (deg)', xlim=(0, 3.4),
           title='The gearbox self-holds 0.13 N·m with torque off; ~1 N·m creeps back at ~2°/s')
    ax.legend(loc='center right')
    return fig


def hunting(th):
    path = servolab.data_dir(SERVO) / 'holdsteps_20261030_003328_gentle-heavy-first.csv.gz'
    df = pd.read_csv(path)
    g = df[df.goal == 511]
    t = g.t_s - g.t_s.iloc[0]
    fig, axes = plt.subplots(2, 1, figsize=(7, 4.2), sharex=True)
    axes[0].plot(t, (g.position - CFG['down_count']) / CPD, color=th['series'][0], lw=0.8, marker='o', ms=2.5)
    axes[0].set_ylabel('angle (deg)')
    axes[1].plot(t, -g.load / 10, color=th['series'][0], lw=0.8, marker='o', ms=2.5)
    axes[1].set(ylabel='output (%)', xlabel='time since the move back to down started (s), 20 Hz telemetry')
    axes[0].set_title('Hanging down (no gravity preload) the position loop hunts in the gear play')
    fig.tight_layout()
    return fig


CHARTS = [
    ('motor_slew', motor_slew),
    ('position_slew', position_slew),
    ('speed_vs_pwm', speed_vs_pwm),
    ('sensor_window', sensor_window),
    ('speed_register', speed_register),
    ('step_response', step_response),
    ('torque_per_pwm', torque_per_pwm),
    ('gearbox_asymmetry', gearbox_asymmetry),
    ('static_hold', static_hold),
    ('heavy_speed_vs_angle', heavy_speed_vs_angle),
    ('regen_clamp', regen_clamp),
    ('bang', bang),
    ('hunting', hunting),
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    only = set(sys.argv[1:])
    for name, fn in CHARTS:
        if only and name not in only:
            continue
        for theme, th in THEMES.items():
            style(th)
            fig = fn(th)
            fig.tight_layout()
            suffix = '' if theme == 'light' else '-dark'
            fig.savefig(OUT / f'{name}{suffix}.png', dpi=150)
            plt.close(fig)
        print(f'rendered {name}')


if __name__ == '__main__':
    main()
