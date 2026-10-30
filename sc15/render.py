#!/usr/bin/env python3
"""Render the SC15 characteristic charts into rendered/charts/.

Each chart is drawn twice, for light and dark page themes. rendered/README.md
references them by name; rerun this after adding data:

    python3 sc15/render.py

The model charts need the fitting dependencies (numba, cma) and the fits in
sc15/model/.
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


BATCH = '093448_heavy-batch'  # the main part of the full heavy batch


def position_law(th):
    df = run(BATCH)
    g = df[(df['mode'] == servolab.POSITION) & (df.speed == 0) & df.valid].copy()
    g['err'] = (g.command - g.position).abs()
    g['out'] = g.load.abs() / 10
    fig, ax = plt.subplots(figsize=(7, 3.8))
    err = np.arange(0, 31)
    for color, kp in zip(th['series'], (8, 15, 30)):
        m = g[g.kp == kp].groupby('err').out.agg(['median', 'count'])
        m = m[(m['count'] >= 30) & (m.index <= 30)]
        law = np.where(err >= (2 if kp == 30 else 3), 3 + kp * (err - 1) / 10, 0)
        ax.plot(err, law, color=color, lw=0.8)
        ax.scatter(m.index, m['median'], s=16, color=color, edgecolors=th['surface'], linewidths=1,
                   label=f'kp {kp}', zorder=3)
    note(ax, th, 18.5, 2, 'lines: 30 + kp × (error − 1) units\n(punch 30, dead band 1)', va='bottom')
    ax.set(xlabel='|goal − position| at rest (counts; 4.47 per degree)', ylabel='applied output (%)',
           xlim=(0, 30.5), ylim=(0, 92),
           title='Position mode at rest: output = punch + kp × (error − dead band), no integral')
    ax.legend(loc='upper left')
    return fig


def friction_band(th):
    df = run(BATCH)
    g = df[df.segment.between(60, 76)]
    t = g.t_s - g.t_s.iloc[0]
    fig, axes = plt.subplots(2, 1, figsize=(7, 4.6), sharex=True)
    axes[0].plot(t, (g.command - CFG['down_count']) / CPD, color=th['ref'], lw=0.8, label='goal')
    axes[0].plot(t, g.deg, color=th['series'][0], lw=LW, label='position')
    axes[0].set_ylabel('angle (deg)')
    axes[0].legend(loc='upper left', ncol=2)
    axes[1].plot(t, -g.load / 10, color=th['series'][0], lw=LW)
    axes[1].set(ylabel='applied output (%)', xlabel='time (s), kp 15, 1.52 kg at 65 mm')
    axes[0].set_title('At 45° (0.69 N·m) the arm does not move while the drive varies 10–30 %')
    fig.tight_layout()
    return fig


def heavy_release(th):
    df = run(BATCH)
    released = df[(df.block == 6) & (df['mode'] == servolab.RELEASED)]
    tg = servolab.gravity_torque_max(CFG, '1.52kg')
    fig, ax = plt.subplots(figsize=(7, 3.8))
    ends = []
    for i, (_, g) in enumerate(released.groupby('segment')):
        g = g[g.valid]
        t = g.t_s - g.t_s.iloc[0]
        ax.plot(t, g.deg, color=th['series'][0], lw=LW)
        note(ax, th, 0, g.deg.iloc[0], f'{g.deg.iloc[0]:+.0f}° ', ha='right', va='center')
        ends.append(g.deg.iloc[-1])
    hold = tg * np.sin(np.radians(np.mean(np.abs(ends)))) * 1000
    for end in (min(ends), max(ends)):
        note(ax, th, 3.0, end, f' {end:+.1f}°', va='center')
    ax.axhline(0, color=th['ref'], lw=0.6)
    ax.set(xlabel='time since torque off (s)', ylabel='angle (deg)', xlim=(-0.45, 3.45),
           title=f'Released, 1.52 kg stops ~9° short of down: friction holds {hold / 1000:.2f} N·m')
    return fig


def thermal_derating(th):
    tg = servolab.gravity_torque_max(CFG, '1.52kg')
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for color, duty in zip(th['series'], (350, 500, 650)):
        xs, ys = [], []
        for key in ('heavy-first-turns', BATCH):
            df = run(key)
            for _, g in df[(df['mode'] == servolab.MOTOR) & (df.command.abs() == duty)].groupby('segment'):
                tail = g[g.t_s > g.t_s.max() - 0.3]
                if not (tail.speed.abs().median() == 0 and tail.valid.all()):
                    continue  # turned or never settled: no stall
                xs.append(tail.temperature_c.median())
                ys.append(tg * abs(np.sin(np.radians(tail.deg.median()))) / duty * 1000)
        ax.scatter(xs, ys, s=30, color=color, edgecolors=th['surface'], linewidths=1,
                   label=f'{duty // 10} % stalled', zorder=3)
    note(ax, th, 27.3, 1.6, 'last night:\n50 and 65 % turned', ha='center')
    ax.set(xlabel='reported temperature at the stall (°C)', ylabel='stall torque per 0.1 % (mN·m)',
           xlim=(22, 42), ylim=(0, 3), title='Motor-mode stall torque drops ~20 % when warm (1.52 kg, from rest)')
    ax.legend(loc='lower left')
    return fig


def temperature(th):
    main = run(BATCH)
    tail = run('094731_heavy-batch')
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for g, t in ((main, main.t_s), (tail, tail.t_s + 761.5)):  # tail: board time minus ~1 s
        g = g[g.valid]
        ax.plot(t[g.index] / 60, g.temperature_c.rolling(250, center=True).median(), color=th['series'][0], lw=LW)
    names = main.attrs['blocks']
    starts = main.groupby('block').t_s.min().sort_values()
    labels = {'holds': 'holds', 'backlash': 'backlash', 'chirp_soft': 'trajectories', 'lift_drop': 'lift-drop',
              'heavy_laps': 'laps', 'chaos': 'chaos'}
    for b, t0 in starts.items():
        name = names.get(b, '')
        if name in labels:
            ax.axvline(t0 / 60, color=th['grid'], lw=0.8)
            note(ax, th, t0 / 60, 64, f' {labels[name]}', va='top', rotation=90)
    ax.axvspan(694.4 / 60, 761.5 / 60, color=th['grid'], alpha=0.6, lw=0)
    note(ax, th, 728 / 60, 23, 'USB\ndropout', ha='center')
    ax.set(xlabel='time in the batch (min)', ylabel='reported temperature (°C, 1 s median)', ylim=(18, 66),
           title='The heavy batch heats the servo from 21 to 51 °C; it cools only once parked')
    return fig


PROBED = '101658_heavy-batch-probed'  # 1.52 kg at 50 mm, 8.7 V


def probe_stalls(df, setup):
    """(time, reported temperature, signed duty, mN·m per unit) per probe push,
    including the probes inside a burn (the only ±250 segments there)."""
    tg = servolab.gravity_torque_max(CFG, setup)
    probes = [b for b, name in df.attrs['blocks'].items() if name in ('probe', 'burn', 'burn_high')]
    rows = []
    pushes = df[df.block.isin(probes) & df['mode'].isin([servolab.MOTOR, servolab.FAULT])
                & (df.command.abs() == 250)]
    for _, g in pushes.groupby('segment'):
        g = g[g.valid]
        tail = g[g.t_s > g.t_s.max() - 0.5]
        rows.append((g.t_s.iloc[0], tail.temperature_c.median(), g.command.iloc[0],
                     tg * abs(np.sin(np.radians(tail.deg.median()))) / abs(g.command.iloc[0]) * 1000))
    return pd.DataFrame(rows, columns=['t', 'temp', 'duty', 'per_unit'])


def voltage_scaling(th):
    keys = [PROBED, 'at-50mm-7.4V', 'at-50mm-6.6V', 'at-50mm-4.8V']
    probes_v, probes_k, laps_v, laps_rpm = [], [], [], []
    for key in keys:
        df = run(key)
        cold = probe_stalls(df, '1.52kg-50mm').iloc[:2]  # first pair, ~27 °C
        probes_v.append(np.mean([df[(df.t_s >= t) & (df.t_s < t + 4) & df.valid].voltage_v.median() for t in cold.t]))
        probes_k.append(cold.per_unit.mean())
        laps = [b for b, name in df.attrs['blocks'].items() if name == 'heavy_laps'][0]
        for _, g in df[(df.block == laps) & (df['mode'] == servolab.MOTOR) & (df.command.abs() == 800)].groupby('segment'):
            ok = g[g.valid]
            p, t = ok.position.to_numpy(), ok.t_s.to_numpy()
            down = CFG['down_count']
            cross = (p[:-1] > down) & (p[1:] <= down) if g.command.iloc[0] > 0 else (p[:-1] < down) & (p[1:] >= down)
            period = np.diff(t[:-1][cross])
            if len(period):
                laps_v.append(0.8 * g.voltage_v.median())
                laps_rpm.append(60 / np.median(period))
    probes_v, probes_k = np.array(probes_v), np.array(probes_k)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8, 3.8))
    k = (probes_v @ probes_k) / (probes_v @ probes_v)
    v = np.array([0, 9])
    a1.plot(v, k * v, color=th['ref'], lw=0.8)
    a1.scatter(probes_v, probes_k, s=30, color=th['series'][0], edgecolors=th['surface'], linewidths=1, zorder=3)
    note(a1, th, 0.4, 2.35, f'proportional:\n{k * 1000:.0f} µN·m per unit per volt', va='top')
    a1.set(xlabel='supply at the probe (V)', ylabel='stall torque per 0.1 % (mN·m)', xlim=(0, 9), ylim=(0, 2.5))
    a1.set_title('Cold 25 % probes', fontsize=9, fontweight='normal', color=th['text2'])
    slope, icpt = np.polyfit(laps_v, laps_rpm, 1)
    v = np.array([-icpt / slope, max(laps_v) + 0.3])
    a2.plot(v, slope * v + icpt, color=th['ref'], lw=0.8)
    a2.scatter(laps_v, laps_rpm, s=30, color=th['series'][0], edgecolors=th['surface'], linewidths=1, zorder=3)
    note(a2, th, -icpt / slope, 2, f'  zero at {-icpt / slope:.1f} V', va='bottom')
    a2.set(xlabel='0.8 × supply under load (V)', ylabel='turn rate (RPM)', xlim=(0, 7.5), ylim=(0, 65))
    a2.set_title('80 % laps (stalled at 4.8 V)', fontsize=9, fontweight='normal', color=th['text2'])
    fig.suptitle('Supply voltage, 1.52 kg at 50 mm: torque scales with it; loaded speed has an offset',
                 x=0.02, ha='left', fontsize=10, fontweight='bold', color=th['text'])
    fig.tight_layout()
    return fig


def probes(th):
    p = probe_stalls(run(PROBED), '1.52kg-50mm')
    labels = ['start', 'after backlash', 'after trajectories', 'after laps + bang', 'after chaos']
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for color, (label, sign) in zip(th['series'], [('pushing toward + angles', -1), ('pushing toward − angles', 1)]):
        g = p[np.sign(p.duty) == sign].reset_index(drop=True)
        ax.plot(g.temp, g.per_unit, color=color, lw=0.8)
        ax.scatter(g.temp, g.per_unit, s=30, color=color, edgecolors=th['surface'], linewidths=1, label=label, zorder=3)
        for i, row in g.iterrows():
            note(ax, th, row.temp, row.per_unit, str(i + 1), va='center',
                 xytext=(6, 0), textcoords='offset points')
    note(ax, th, 25.5, 1.42, '  '.join(f'{i + 1} {label}' for i, label in enumerate(labels)), va='bottom')
    ax.set(xlabel='reported temperature (°C)', ylabel='stall torque per 0.1 % (mN·m)', ylim=(1.35, 2.6),
           title='25 % probes: torque falls as it heats, and the sensor lags a hard block')
    ax.legend(loc='upper right')
    return fig


MODEL_DIR = servolab.servo_dir(SERVO) / 'model'
# Validation MAE (rad) read off Fig. 5 of Duclusaud et al., ICRA 2025, ±0.0005.
BAM_FIG5 = {'MX-64': [0.0273, 0.0252, 0.0213, 0.0139, 0.0093, 0.0093],
            'MX-106': [0.0192, 0.0185, 0.0145, 0.0105, 0.0095, 0.0095]}


def model_mae(th):
    import json
    ours = [json.loads((MODEL_DIR / f'bam-m{i}.json').read_text())['val_mae_rad'] for i in range(1, 7)]
    fig, ax = plt.subplots(figsize=(8, 3.6))
    x = np.arange(6)
    series = [('SC15, held-out runs', ours)] + [(f'BAM {k} (paper)', v) for k, v in BAM_FIG5.items()]
    for j, (color, (label, v)) in enumerate(zip(th['series'], series)):
        ax.bar(x + (j - 1) * 0.27, np.degrees(v), 0.25, color=color, label=label, edgecolor=th['surface'], linewidth=1)
    for xi, v in zip(x, ours):
        note(ax, th, xi - 0.27, np.degrees(v) + 0.03, f'{np.degrees(v):.2f}', ha='center', va='bottom')
    ax.set_xticks(x, ['M1', 'M2', 'M3', 'M4', 'M5', 'M6'])
    ax.set(ylabel='position MAE, 6 s windows (°)', ylim=(0, 1.8))
    ax.grid(axis='x', visible=False)
    ax.legend(loc='upper right')
    ax.set_title('Friction models, BAM protocol: the SC15 fit lands between BAM\'s M1 and best')
    return fig


def model_fit(th):
    import json
    import fit_model
    import sc_model
    fit = json.loads((MODEL_DIR / 'full-m6.json').read_text())
    p = sc_model.full_vector('m6', fit['params'], thermal=fit['thermal'])
    ws = [w for w in fit_model.load_windows() if w['run'] in fit_model.VALIDATION_RUNS]
    want = [('trajectory', 'dual_sine_soft', 15), ('trajectory', 'chirp_soft', 30), ('chaos', 'chaos', 15)]
    picks = []
    for kind, block, kp in want:  # the window with the median error of its kind
        cand = [w for w in ws if w['kind'] == kind and w['block'] == block and w['kp'] == kp and len(w['mode']) == 1500]
        e = fit_model.window_errors(p, fit_model.pack(cand), True)
        picks.append(cand[int(np.argsort(e)[len(e) // 2])])
    pk = fit_model.pack(picks)
    err = fit_model.window_errors(p, pk, True)
    fig, axs = plt.subplots(3, 1, figsize=(8, 7.2), sharex=True)
    for i, (ax, w) in enumerate(zip(axs, picks)):
        a, n = pk['starts'][i], pk['lengths'][i]
        sim = np.empty(n)
        sc_model.rollout(p, pk['mode'][a:a + n], pk['command'][a:a + n], pk['command_time'][a:a + n],
                         pk['flags'][a:a + n], pk['load'][a:a + n], pk['volt'][a:a + n], pk['tempc'][a:a + n],
                         pk['kp'][i], pk['mgl'][i], pk['jload'][i], pk['spool'][i], pk['q0'][i], pk['dq0'][i],
                         pk['u0'][i], True, sim, pk['meas'][a:a + n], pk['meas_vel'][a:a + n], pk['reset'][i])
        t = np.arange(n) * sc_model.TICK
        if w['kind'] == 'trajectory':
            ax.plot(t, (w['command'] - CFG['down_count']) / CPD, color=th['ref'], lw=0.6, label='goal')
        ax.plot(t, np.degrees(pk['meas'][a:a + n]), color=th['series'][0], label='measured')
        ax.plot(t, np.degrees(sim), color=th['series'][1], label='simulated')
        what = f"{w['block']} at kp {w['kp']}" if w['kind'] == 'trajectory' else 'chaos (motor and position segments, re-synced every 1 s)'
        ax.set_title(f"{what}: MAE {np.degrees(err[i]):.2f}°", fontsize=9, fontweight='normal', color=th['text2'])
        ax.set_ylabel('angle (°)')
    axs[0].legend(loc='upper right', ncol=3)
    axs[-1].set_xlabel('time (s)')
    fig.suptitle('Full model (M6, firmware controller, thermal) on held-out runs, 1.52 kg at 50 mm',
                 x=0.02, ha='left', fontsize=10, fontweight='bold', color=th['text'])
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
    ('position_law', position_law),
    ('friction_band', friction_band),
    ('heavy_release', heavy_release),
    ('thermal_derating', thermal_derating),
    ('temperature', temperature),
    ('probes', probes),
    ('voltage_scaling', voltage_scaling),
    ('model_mae', model_mae),
    ('model_fit', model_fit),
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
