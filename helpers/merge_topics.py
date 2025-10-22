#!/usr/bin/env python3
"""
Merge several ulog-exported per-topic CSVs into one cleaned, timestamped CSV.

Usage examples:
  python3 merge_topics.py --hz 20 --out merged_replay.csv \
      02_41_26.vehicle_global_position.csv \
      02_41_26.vehicle_local_position.csv \
      02_41_26.vehicle_attitude.csv \
      02_41_26.vehicle_gps_position.csv \
      02_41_26.vehicle_imu.csv

To keep IMU at original rate and not interpolate it into the master file:
  python3 merge_topics.py --hz 20 --out merged_replay.csv --keep-highrate 02_41_26.vehicle_imu.csv \
      <other csvs...>
"""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import sys

def guess_timestamp_col(df):
    # heuristics for timestamp column
    candidates = [c for c in df.columns if 'timestamp' in c.lower() or c.lower().endswith('_us') or c.lower().endswith('us') or c.lower().endswith('time')]
    if candidates:
        # prioritize exact 'timestamp' or column starting with '_timestamp'
        for c in candidates:
            if c.lower() == 'timestamp' or c.lower().startswith('_timestamp'):
                return c
        return candidates[0]
    # fallback: first numeric column if it looks like microseconds
    for c in df.columns:
        if np.issubdtype(df[c].dtype, np.number):
            return c
    # last fallback
    return df.columns[0]

def load_csv_with_ts(path: Path):
    df = pd.read_csv(path, header=0, low_memory=False)
    ts_col = guess_timestamp_col(df)
    # convert to float
    try:
        ts = df[ts_col].astype(float)
    except Exception:
        # if column not numeric, try removing non-numeric characters
        ts = pd.to_numeric(df[ts_col], errors='coerce').astype(float)
    # if timestamps are very large assume microseconds
    if ts.max() > 1e6:
        t_s = ts * 1e-6
    else:
        t_s = ts
    df = df.copy()
    df['__time_s'] = t_s
    df = df.set_index('__time_s', drop=True)
    # replace inf with NaN
    df = df.replace([np.inf, -np.inf], np.nan)
    return df

def main():
    p = argparse.ArgumentParser()
    p.add_argument('csvs', nargs='+', help='topic CSV files to merge')
    p.add_argument('--hz', type=float, default=20.0, help='target sample rate for merged CSV (Hz)')
    p.add_argument('--out', default='merged_replay.csv', help='output merged CSV')
    p.add_argument('--keep-highrate', nargs='*', default=[], help='list of filenames to leave at original rate (not merged)')
    p.add_argument('--min-coverage', type=float, default=0.0, help='drop master rows if less than this fraction of core columns present (0..1)')
    p.add_argument('--drop-thresh', type=float, default=0.9,
                   help='drop columns with > this fraction of NaN (0..1). Set to 1.0 to drop none.')
    args = p.parse_args()

    files = [Path(x) for x in args.csvs]
    keep_highrate = set(Path(x).name for x in args.keep_highrate)

    dfs = {}
    tmin, tmax = np.inf, 0.0
    for f in files:
        if not f.exists():
            print(f"ERROR: file not found: {f}", file=sys.stderr)
            sys.exit(2)
        df = load_csv_with_ts(f)
        if df.index.duplicated().any():
            print(f"NOTE: duplicate timestamps in {f.name}; aggregating...", file=sys.stderr)
            # numeric columns -> mean; non-numeric -> first (or you can choose other policy)
            numeric = df.select_dtypes(include=[np.number]).groupby(level=0).mean()
            nonnum = df.select_dtypes(exclude=[np.number]).groupby(level=0).first()
            # join back numeric+non-numeric (numeric may be empty or vice-versa)
            if len(numeric.columns) and len(nonnum.columns):
                df = numeric.join(nonnum)
            elif len(numeric.columns):
                df = numeric
            else:
                df = nonnum
        dfs[f.name] = df
        tmin = min(tmin, df.index.min())
        tmax = max(tmax, df.index.max())

    if tmin >= tmax:
        print("ERROR: timestamps invalid or single-point", file=sys.stderr)
        sys.exit(2)

    dt = 1.0 / args.hz
    master_idx = np.arange(tmin, tmax + dt/2.0, dt)
    master = pd.DataFrame(index=master_idx)

    saved_highrate = []
    for name, df in dfs.items():
        numeric = df.select_dtypes(include=[np.number]).copy()
        if numeric.shape[1] == 0:
            print(f"NOTE: no numeric columns in {name}, skipping", file=sys.stderr)
            continue

        # Save original high-rate file if requested
        if name in keep_highrate:
            # write this out raw (clean inf->NaN)
            outname = f"highrate_{name}"
            numeric.to_csv(outname, index_label='time_s')
            saved_highrate.append(outname)
            print(f"WROTE high-rate stream (no resample): {outname}")
            continue

        # reindex/interpolate onto master index
        # reindex to master and interpolate numeric columns
        numeric = numeric.reindex(master.index).interpolate(method='linear', limit_direction='both')
        # prefix columns with topic name (without extension)
        prefix = Path(name).stem
        numeric.columns = [f"{prefix}::{c}" for c in numeric.columns]
        master = master.join(numeric)

    # Optional: drop rows with too little coverage
    if args.min_coverage > 0:
        numeric_cols = master.columns
        non_na_count = master[numeric_cols].notna().sum(axis=1)
        frac_present = non_na_count / len(numeric_cols)
        keep_mask = frac_present >= args.min_coverage
        master = master.loc[keep_mask]
        # ---------- POST-MERGE CLEANING (new) ----------
    # configurable drop threshold and cleaning policies
    drop_thresh = 0.9  # fraction of rows that are NaN to drop a column
    if '--drop-thresh' in ' '.join(sys.argv):
        try:
            drop_thresh = float([a for a in sys.argv if a.startswith('--drop-thresh=')][0].split('=')[1])
        except Exception:
            pass

    # compute NaN fraction per column
    nrows = len(master)
    nan_frac = master.isna().sum() / float(nrows)

    # decide which columns to drop
    drop_cols = nan_frac[nan_frac > drop_thresh].index.tolist()
    if drop_cols:
        print(f"NOTE: dropping {len(drop_cols)} columns with >{int(drop_thresh*100)}% NaN", file=sys.stderr)
        # optional: list them (comment/uncomment if verbose desired)
        # for c in drop_cols: print("  drop:", c, file=sys.stderr)
        master = master.drop(columns=drop_cols)

    # Interpolate numeric columns, fill non-numeric
    numcols = master.select_dtypes(include=[np.number]).columns
    objcols = master.columns.difference(numcols)

    if len(numcols):
        master[numcols] = master[numcols].interpolate(method='linear', limit_direction='both')

    if len(objcols):
        master[objcols] = master[objcols].ffill().bfill()

    # Optionally write cleaned version too
    cleaned_out = args.out.replace('.csv', '.cleaned.csv')
    master.to_csv(cleaned_out, index=True, float_format='%.9f')
    print("WROTE cleaned merged CSV:", cleaned_out, file=sys.stderr)
    # ---------- end cleaning ----------

    master.index.name = 'time_s'
    master.to_csv(args.out, index=True, float_format='%.9f')
    print("WROTE merged CSV:", args.out)
    if saved_highrate:
        print("Saved high-rate files:", saved_highrate)

if __name__ == "__main__":
    main()
