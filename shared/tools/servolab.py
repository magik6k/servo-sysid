"""Shared helpers for servo measurement runs.

Layout: <repo>/<servo>/servo.json (geometry and sensor constants),
<repo>/<servo>/data/<run>.csv.gz plus <run>.meta.txt (the firmware's
'#' preamble and notes). Derived files go to <repo>/<servo>/data/derived/.
"""

import json
import pathlib
import re

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

REPO = pathlib.Path(__file__).resolve().parents[2]

MOTOR, POSITION, COOLDOWN, FAULT, RELEASED = range(5)
FLAG_SEGMENT_START = 0x01
FLAG_MODE_ENTRY = 0x02
FLAG_READ_FAILED = 0x04


def servo_dir(servo):
    return REPO / servo


def data_dir(servo):
    return servo_dir(servo) / 'data'


def config(servo):
    return json.loads((servo_dir(servo) / 'servo.json').read_text())


def run_stem(path):
    """chaos_<stamp>_<tag> for <stem>.csv, <stem>.csv.gz or <stem>.meta.txt."""
    name = pathlib.Path(path).name
    for suffix in ('.csv.gz', '.csv', '.meta.txt'):
        if name.endswith(suffix):
            return name[:-len(suffix)]
    return name


def meta_path(path):
    path = pathlib.Path(path)
    return path.with_name(run_stem(path) + '.meta.txt')


def find_run(servo, key):
    """The one raw recording in <servo>/data whose name contains key."""
    hits = sorted(p for p in data_dir(servo).glob('*.csv.gz') if key in p.name)
    if len(hits) != 1:
        raise SystemExit(f'{len(hits)} runs match {key!r} in {data_dir(servo)}')
    return hits[0]


def read_meta(path):
    p = meta_path(path)
    return p.read_text().splitlines() if p.exists() else []


def block_names(meta):
    for line in meta:
        if line.startswith('# blocks:'):
            return {int(k): v for k, v in re.findall(r'(\d+)=(\w+)', line)}
    return {}


def load(servo, key):
    """Raw run as a DataFrame plus angle and derivative columns."""
    path = find_run(servo, key)
    df = pd.read_csv(path)
    add_kinematics(df, config(servo))
    df.attrs['run'] = run_stem(path)
    df.attrs['blocks'] = block_names(read_meta(path))
    return df


def add_kinematics(df, cfg):
    """deg from down, and vel/acc (deg/s, deg/s²) within trusted runs only.

    The potentiometer is only trusted inside cfg trust window, and its
    reading sweeps the whole range in 1-3 samples at the track gap, so any
    step over max_step counts per sample also breaks a run.
    """
    cpd = cfg['counts_per_rev'] / 360
    lo, hi = cfg['trust_counts']
    dt = cfg['sample_s']
    df['deg'] = (df.position - cfg['down_count']) / cpd
    step = df.position.diff().abs()
    df['valid'] = (df.position.between(lo, hi)
                   & (step <= cfg['max_step_counts'])
                   & (step.shift(-1) <= cfg['max_step_counts']))
    df['vel'] = np.nan
    df['acc'] = np.nan
    runs = (~df.valid).cumsum()
    for _, g in df[df.valid].groupby(runs[df.valid]):
        if len(g) < 17:
            continue
        deg = g.deg.to_numpy(float)
        df.loc[g.index[4:-4], 'vel'] = savgol_filter(deg, 9, 2, deriv=1, delta=dt)[4:-4]
        df.loc[g.index[7:-7], 'acc'] = savgol_filter(deg, 15, 2, deriv=2, delta=dt)[7:-7]


def gravity_torque_max(cfg, setup):
    """Peak gravity torque (N·m) of a pendulum setup from servo.json."""
    s = cfg['setups'][setup]
    return 9.81 * (s['mass_kg'] * s['radius_m'] + s['arm_kg'] * s['arm_com_m'])
