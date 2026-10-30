#!/usr/bin/env python3
"""Split a chaos v2 batch recording into one CSV per block (and P gain).

Block names come from the '# blocks:' line in the run's .meta.txt.
Writes <run>.<block>[.kpN].csv into derived/ next to the input (not kept
in git: rerun this to regenerate) and prints a summary.
"""

import argparse
import pathlib

import pandas as pd

import servolab

TRAJECTORY_BLOCKS = {'chirp', 'chirp_soft', 'dual_sine', 'dual_sine_soft', 'raise_lower'}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv', type=pathlib.Path)
    args = ap.parse_args()

    names = servolab.block_names(servolab.read_meta(args.csv))
    if not names:
        raise SystemExit(f'No "# blocks:" line in {servolab.meta_path(args.csv)}')
    df = pd.read_csv(args.csv)
    out_dir = args.csv.parent / 'derived'
    out_dir.mkdir(exist_ok=True)
    stem = servolab.run_stem(args.csv)

    groups = list(df.groupby(['block', 'kp'], sort=False))
    gains_per_block = pd.Series([block for (block, _), _ in groups]).value_counts()

    for (block, kp), part in groups:
        name = names.get(block, f'block{block}')
        tag_gain = name in TRAJECTORY_BLOCKS or gains_per_block[block] > 1
        suffix = f'.{name}.kp{kp}' if tag_gain else f'.{name}'
        out = out_dir / (stem + suffix + '.csv')
        part.to_csv(out, index=False)
        print(f'{out.name:70s} {len(part):6d} frames  {part.t_s.iloc[-1] - part.t_s.iloc[0]:6.1f} s  '
              f'modes {sorted(part["mode"].unique().tolist())}')


if __name__ == '__main__':
    main()
