"""
Usage:
  python3 ulog_export_topic.py path/to/file.ulg [topic1 topic2 ...]

Exports the requested topics to csv, if no topic is given nothing is exported 
"""
import sys
from pathlib import Path
import pandas as pd
import numpy as np
from pyulog import ULog

if len(sys.argv) < 2:
    print("Usage: python3 ulog_export_topic.py path/to/file.ulg [topic1 topic2 ...]")
    sys.exit(1)

ulg_path = Path(sys.argv[1])
u = ULog(str(ulg_path))

print("Available topics in ULog:")
avail = [d.name for d in u.data_list]
for name in avail:
    print(" -", name)

# topics supplied on cmdline? otherwise use a safe default set to try.
if len(sys.argv) > 2:
    topics = sys.argv[2:]
else:
    preferred = ["vehicle_global_position", "vehicle_local_position", "sensor_gps", "sensor_combined", "battery_status"]
    topics = [t for t in preferred if t in avail]
    if not topics:
        topics = avail  # fallback to exporting anything present

out_dir = ulg_path.with_suffix('').with_name(ulg_path.stem + "_csv")
out_dir.mkdir(parents=True, exist_ok=True)

def convert_dataset(ds):
    """
    Accepts:
      - pyulog.core.ULog.Data object (has .data)
      - numpy structured array / recarray
      - dict-of-arrays
      - list-of-dicts
    Returns (df, reason) where df is pandas.DataFrame or None and reason explains failure.
    """
    # unwrap pyulog Data object
    if hasattr(ds, "data") and not isinstance(ds, (dict, list, np.ndarray, pd.DataFrame)):
        ds_inner = ds.data
    else:
        ds_inner = ds

    # If it's a dict of arrays
    if isinstance(ds_inner, dict):
        try:
            return pd.DataFrame({k: np.asarray(v) for k, v in ds_inner.items()}), "dict"
        except Exception as e:
            return None, f"dict->DataFrame failed: {e}"

    # numpy structured array / recarray
    if hasattr(ds_inner, "dtype") and getattr(ds_inner.dtype, "names", None):
        try:
            # build frame from fields
            return pd.DataFrame({name: ds_inner[name] for name in ds_inner.dtype.names}), "structured_array"
        except Exception as e:
            try:
                # fallback: build row-by-row
                rows = [dict(zip(ds_inner.dtype.names, row)) for row in ds_inner]
                return pd.DataFrame(rows), "structured_array->rows"
            except Exception as e2:
                return None, f"struct->DataFrame failed: {e} / {e2}"

    # pandas-friendly (list of dicts or 2D numeric)
    try:
        return pd.DataFrame(ds_inner), "pd-friendly"
    except Exception as e:
        return None, f"generic DataFrame failed: {e}"

# process each requested topic
exported = []
for topic in topics:
    print("\n---\nProcessing topic:", topic)
    # find matching entry in u.data_list
    entry = None
    for d in u.data_list:
        if d.name == topic:
            entry = d
            break
    if entry is None:
        print("  Topic not found in ULog. Skipping.")
        continue

    print("  Found entry type:", type(entry))
    # convert to dataframe
    df, reason = convert_dataset(entry)
    if df is None:
        print("  Could not convert topic to DataFrame. Reason:", reason)
        # print a short diagnostic of what the entry contains
        try:
            print("  entry attributes:", [k for k in dir(entry) if not k.startswith("_")][:50])
            if hasattr(entry, "data"):
                print("  entry.data type:", type(entry.data))
                try:
                    # show dtype names if available
                    if hasattr(entry.data, "dtype"):
                        print("  entry.data.dtype.names:", getattr(entry.data.dtype, "names", None))
                except Exception:
                    pass
        except Exception:
            pass
        continue

    # Print fields and small preview
    print("  Converted to DataFrame. columns:", list(df.columns))
    print("  Preview (first 5 rows):")
    print(df.head().to_string(index=False))
    # Export CSV
    out_path = out_dir / f"{ulg_path.stem}.{topic}.csv"
    df.to_csv(out_path, index=False)
    print("  Wrote", out_path, "rows:", len(df))
    exported.append(str(out_path.name))

print("\nExport complete. Files written to:", out_dir)
if exported:
    print("Exported topics:", exported)
else:
    print("No topics exported successfully. If you expected certain topics, try passing them explicitly on the command line.")
