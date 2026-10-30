#!/usr/bin/env python3
"""Fits the SC15 actuator model (sc_model.py) to recorded windows, BAM-style.

CMA-ES minimizes the mean absolute angle error of simulated against
measured windows (model_data.py), closed loop through the firmware
controller unless --open-loop, which replays the logged PWM instead.
Held-out runs score the fit. Writes <servo>/model/<name>.json.

  fit_model.py --model m4 --kinds trajectory drop --name bam-m4
  fit_model.py --model m4 --thermal --kinds trajectory drop motor chaos --name full-m4
"""

import argparse
import json
import pickle
import time

import cma
import numpy as np
import pandas as pd

import model_data
import sc_model
import servolab

VALIDATION_RUNS = ['134414_heavy-batch-burn', '112510_heavy-batch-probed']
# Open-loop motor-mode windows drift with any speed error, so they are
# re-synced to the measurement every second (BAM's reset_period).
RESET_KINDS = {'motor', 'chaos'}
RESET_TICKS = 250
CACHE = servolab.data_dir('sc15') / 'derived' / 'model_windows.pkl'


def load_windows():
    if CACHE.exists():
        return pickle.loads(CACHE.read_bytes())
    ws = model_data.all_windows()
    CACHE.parent.mkdir(parents=True, exist_ok=True)
    CACHE.write_bytes(pickle.dumps(ws))
    return ws


def pack(ws):
    """Flat arrays for sc_model.score."""
    cfg = servolab.config('sc15')
    lengths = np.array([len(w['mode']) for w in ws], np.int64)
    starts = np.r_[0, np.cumsum(lengths)[:-1]].astype(np.int64)
    cat = lambda key, dt: np.concatenate([w[key] for w in ws]).astype(dt)
    setup = [cfg['setups'][w['setup']] for w in ws]
    meas = (cat('pos', float) - sc_model.DOWN_COUNT) / sc_model.COUNTS_PER_RAD
    valid = cat('valid', bool) & np.isfinite(meas)
    meas_vel = np.radians(cat('vel', float))
    meas_vel[~valid] = np.nan
    return dict(
        starts=starts, lengths=lengths,
        mode=cat('mode', np.int64), command=cat('command', float), command_time=cat('command_time', float), flags=cat('flags', np.int64),
        load=cat('load', float), volt=cat('volt', float), tempc=cat('tempc', float),
        meas=np.where(valid, meas, np.nan), valid=valid, meas_vel=meas_vel,
        reset=np.array([RESET_TICKS if w['kind'] in RESET_KINDS else 0 for w in ws], np.int64),
        kinds=np.array([w['kind'] for w in ws]),
        kp=np.array([w['kp'] for w in ws], float),
        mgl=np.array([9.81 * (s['mass_kg'] * s['radius_m'] + s['arm_kg'] * s['arm_com_m']) for s in setup]),
        jload=np.array([s['mass_kg'] * s['radius_m'] ** 2 + s['arm_kg'] * s['arm_com_m'] ** 2 for s in setup]),
        spool=np.array([1.0 if w['setup'].startswith('1.52kg') else 0.0 for w in ws]),
        q0=np.radians([w['q0'] for w in ws]), dq0=np.radians([w['dq0'] for w in ws]),
        u0=np.array([w['u0'] for w in ws], float),
    )


def window_errors(p, packed, closed_loop):
    err = np.empty(len(packed['starts']))
    a = packed
    sc_model.score(p, a['starts'], a['lengths'], a['mode'], a['command'], a['command_time'], a['flags'], a['load'],
                   a['volt'], a['tempc'], a['meas'], a['valid'], a['meas_vel'], a['kp'], a['mgl'],
                   a['jload'], a['spool'], a['q0'], a['dq0'], a['u0'], a['reset'], closed_loop, err)
    return err


INITIAL = dict(k=0.26, b=0.3, J_m=0.008, J_spool=0.001, offset=0.0, aT=0.005, kd=72.0,
               Kc=0.05, Kv=0.05, Kcs=0.03, vs=0.2, alpha=2.0, Km=0.05, Ke=0.05,
               Kms=0.02, Kes=0.02, Kmq=0.01, Keq=0.01, k_dir=0.0)


def to_x(name, v):
    _, lo, hi, log = sc_model.PARAMS[sc_model.IDX[name]]
    return (np.log(v / lo) / np.log(hi / lo)) if log else (v - lo) / (hi - lo)


def from_x(name, x):
    _, lo, hi, log = sc_model.PARAMS[sc_model.IDX[name]]
    x = min(max(x, 0.0), 1.0)
    return lo * (hi / lo) ** x if log else lo + x * (hi - lo)


def fit(model, packed, closed_loop, thermal, directional, evals, seed, start=None, weigh_kinds=False, fixed=None):
    fixed = fixed or {}
    free = [n for n in list(sc_model.MODELS[model]) + (['aT'] if thermal else []) + (['k_dir'] if directional else [])
            if n not in fixed]
    x0 = [to_x(n, (start or INITIAL)[n] if n in (start or {}) else INITIAL[n]) for n in free]
    to_p = lambda x: sc_model.full_vector(model, {**fixed, **{n: from_x(n, v) for n, v in zip(free, x)}}, thermal, directional)
    kinds = packed['kinds']

    def cost(x):  # mean over windows, or over kinds with each kind weighing the same
        e = window_errors(to_p(x), packed, closed_loop)
        if not weigh_kinds:
            return float(np.nanmean(e))
        return float(np.mean([np.nanmean(e[kinds == k]) for k in np.unique(kinds)]))
    es = cma.CMAEvolutionStrategy(x0, 0.15, {'bounds': [0, 1], 'popsize': 24, 'maxfevals': evals,
                                             'seed': seed, 'verbose': -9})
    while not es.stop():
        xs = es.ask()
        es.tell(xs, [cost(x) for x in xs])
    best = es.result.xbest
    return {**fixed, **{n: from_x(n, v) for n, v in zip(free, best)}}, to_p(best), es.result.fbest, es.result.evaluations


def summarize(ws, err):
    t = pd.DataFrame([{k: w[k] for k in ('run', 'setup', 'kind', 'kp')} for w in ws])
    t['mae_deg'] = np.degrees(err)
    return t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default='m4', choices=sorted(sc_model.MODELS))
    ap.add_argument('--kinds', nargs='+', default=['trajectory', 'drop'])
    ap.add_argument('--thermal', action='store_true')
    ap.add_argument('--fix', nargs='*', default=[], metavar='NAME=VALUE', help='hold parameters at these values')
    ap.add_argument('--weigh-kinds', action='store_true', help='each kind of window weighs the same')
    ap.add_argument('--directional', action='store_true', help='torque constant differs by direction')
    ap.add_argument('--open-loop', action='store_true', help='replay the logged PWM')
    ap.add_argument('--evals', type=int, default=6000)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--seeds', type=int, default=1, help='repeat from seed, seed+1, ...; keep the best training score')
    ap.add_argument('--start', help='json of a previous fit to start from')
    ap.add_argument('--name', required=True)
    args = ap.parse_args()

    ws = [w for w in load_windows() if w['kind'] in args.kinds]
    train = [w for w in ws if w['run'] not in VALIDATION_RUNS]
    val = [w for w in ws if w['run'] in VALIDATION_RUNS]
    pt, pv = pack(train), pack(val)
    start = json.loads(open(args.start).read())['params'] if args.start else None
    fixed = {k: float(v) for k, v in (f.split('=') for f in args.fix)}

    t0 = time.time()
    runs = [fit(args.model, pt, not args.open_loop, args.thermal, args.directional, args.evals, args.seed + i, start,
                args.weigh_kinds, fixed) for i in range(args.seeds)]
    params, p, best, n = min(runs, key=lambda r: r[2])
    n = sum(r[3] for r in runs)
    et, ev = window_errors(p, pt, not args.open_loop), window_errors(p, pv, not args.open_loop)
    st, sv = summarize(train, et), summarize(val, ev)
    by = pd.concat([st.assign(split='train'), sv.assign(split='val')])
    table = by.groupby(['split', 'kind']).mae_deg.agg(['mean', 'count']).round(3)
    out = dict(
        name=args.name, model=args.model, fixed=fixed, seeds=args.seeds, seed_costs=[r[2] for r in runs], thermal=args.thermal, directional=args.directional, weigh_kinds=args.weigh_kinds, closed_loop=not args.open_loop,
        kinds=args.kinds, validation_runs=VALIDATION_RUNS, evaluations=n, seconds=round(time.time() - t0),
        params=params,
        train_mae_rad=float(np.nanmean(et)), val_mae_rad=float(np.nanmean(ev)),
        by_kind={f'{s}/{k}': dict(mae_deg=float(r['mean']), windows=int(r['count'])) for (s, k), r in table.iterrows()},
        by_setup_kp={'/'.join(map(str, k)): float(v)
                     for k, v in by.groupby(['split', 'setup', 'kind', 'kp']).mae_deg.mean().round(3).items()},
    )
    path = servolab.servo_dir('sc15') / 'model' / f'{args.name}.json'
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(out, indent=1) + '\n')
    print(f'{args.name}: train {np.degrees(out["train_mae_rad"]):.3f} deg ({out["train_mae_rad"]:.4f} rad), '
          f'val {np.degrees(out["val_mae_rad"]):.3f} deg ({out["val_mae_rad"]:.4f} rad), {n} evals, {out["seconds"]} s')
    print(table.to_string())


if __name__ == '__main__':
    main()
