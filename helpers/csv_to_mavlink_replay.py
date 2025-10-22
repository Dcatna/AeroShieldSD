#!/usr/bin/env python3
"""
Replay merged CSV by publishing MAVLink messages (ATTITUDE, LOCAL_POSITION_NED and GLOBAL_POSITION_INT).

This version is tailored to pymavlink variations — it constructs the MAVLink_global_position_int_message
with the signature found on your machine:
 (time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg)

Usage:
  pip3 install pandas numpy pymavlink
  python3 csv_to_mavlink_replay.py merged_replay.cleaned.csv --target udp:127.0.0.1:14550

Flags:
  --dry         : detect columns but don't send
  --loop        : replay continuously
  --scale S     : time scale factor (1.0 realtime)
  --start-delay : seconds to wait before starting
  --verbose N   : print first N rows of what will be sent (default 10)
"""
import argparse
import time
import sys
import math
import pandas as pd
import numpy as np
from pymavlink import mavutil

def find_col(df, patterns):
    if df is None:
        return None
    for p in patterns:
        for c in df.columns:
            if p.lower() in c.lower():
                return c
    return None

def latlon_to_global_int(lat, lon, alt_m):
    lat_i = int(round(float(lat) * 1e7))
    lon_i = int(round(float(lon) * 1e7))
    alt_mm = int(round(float(alt_m) * 1000.0))
    return lat_i, lon_i, alt_mm

def safe_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default

def safe_int(v, default=0):
    try:
        return int(round(float(v)))
    except Exception:
        return default

def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv', help='cleaned merged CSV file (first column = time seconds)')
    p.add_argument('--target', default='udp:127.0.0.1:14550', help='mavlink target URI')
    p.add_argument('--dry', action='store_true', help='dry run - detect columns only')
    p.add_argument('--loop', action='store_true', help='loop replay')
    p.add_argument('--scale', type=float, default=1.0, help='time scale (1.0 realtime)')
    p.add_argument('--start-delay', type=float, default=0.0, help='delay (s) before starting')
    p.add_argument('--verbose', type=int, default=10, help='print first N rows of planned sends')
    args = p.parse_args()

    # load CSV (index is expected to be time in seconds)
    try:
        df = pd.read_csv(args.csv, index_col=0)
    except Exception as e:
        print("ERROR: failed to read CSV:", e)
        sys.exit(1)

    try:
        t_values = df.index.astype(float).to_numpy()
    except Exception as e:
        print("ERROR: CSV index must be numeric timestamps (seconds).", e)
        sys.exit(1)

    # column detection heuristics
    lat_col = find_col(df, ['latitude', 'vehicle_gps_position::latitude'])
    lon_col = find_col(df, ['longitude', 'vehicle_gps_position::longitude'])
    alt_col = find_col(df, ['altitude_msl', 'altitude_msl_m', 'absolute_altitude', 'altitude', 'alt'])
    rel_alt_col = find_col(df, ['relative_alt', 'relative_altitude', 'relative_alt_m'])

    x_col = find_col(df, ['vehicle_local_position::x','local_position::x','::x',' local_x','local_x'])
    y_col = find_col(df, ['vehicle_local_position::y','local_position::y','::y',' local_y','local_y'])
    z_col = find_col(df, ['vehicle_local_position::z','local_position::z','::z',' local_z','local_z'])

    q0 = find_col(df, ['vehicle_attitude::q[0]','::q[0]','::q0',' q0',' qw'])
    q1 = find_col(df, ['vehicle_attitude::q[1]','::q[1]','::q1',' q1',' qx'])
    q2 = find_col(df, ['vehicle_attitude::q[2]','::q[2]','::q2',' q2',' qy'])
    q3 = find_col(df, ['vehicle_attitude::q[3]','::q[3]','::q3',' q3',' qz'])

    vx_col = find_col(df, ['::vx','vehicle_local_position::vx',' vx','v_x'])
    vy_col = find_col(df, ['::vy','vehicle_local_position::vy',' vy','v_y'])
    vz_col = find_col(df, ['::vz','vehicle_local_position::vz',' vz','v_z'])

    speed_col = find_col(df, ['speed','groundspeed','gndspeed'])

    print("Detected columns:")
    print(" lat/lon/alt:", lat_col, lon_col, alt_col, "(rel_alt_col:", rel_alt_col, ")")
    print(" local x,y,z:", x_col, y_col, z_col)
    print(" quats:", q0, q1, q2, q3)
    print(" vels:", vx_col, vy_col, vz_col)
    print(" speed:", speed_col)

    if args.dry:
        print("Dry run - exiting without sending.")
        return

    # connect
    print("Connecting to", args.target)
    master = mavutil.mavlink_connection(args.target)
    try:
        master.wait_heartbeat(timeout=5)
    except Exception as e:
        print("Warning: heartbeat wait failed or timed out:", e)
    else:
        print("Heartbeat from system (connected).")

    if args.start_delay > 0:
        print(f"Waiting {args.start_delay}s before starting...")
        time.sleep(args.start_delay)

    # helper to compute yaw degrees from quaternion
    def quat_to_yaw_deg(qw, qx, qy, qz):
        ysqr = qy * qy
        t3a = +2.0 * (qw * qz + qx * qy)
        t4a = +1.0 - 2.0 * (ysqr + qz * qz)
        yaw = np.arctan2(t3a, t4a)
        return (np.degrees(yaw) + 360.0) % 360.0

    # preview first N rows
    N = min(args.verbose, len(t_values))
    print(f"\nPreview first {N} rows (time, lat, lon, alt_m, rel_alt_m, vx,vy,vz, yaw_deg):")
    for i in range(N):
        r = df.iloc[i]
        lat = r.get(lat_col, None)
        lon = r.get(lon_col, None)
        alt = r.get(alt_col, None)
        rel = r.get(rel_alt_col, None)
        vx = r.get(vx_col, None)
        vy = r.get(vy_col, None)
        vz = r.get(vz_col, None)
        yaw_deg = None
        try:
            if q0 and q1 and q2 and q3:
                yaw_deg = quat_to_yaw_deg(float(r[q0]), float(r[q1]), float(r[q2]), float(r[q3]))
        except Exception:
            yaw_deg = None
        print(float(t_values[i]), lat, lon, alt, rel, vx, vy, vz, yaw_deg)
    print("---- starting replay ----\n")

    # main loop
    t0 = float(t_values[0])
    n = len(t_values)

    while True:
        start_time = time.time()
        for idx in range(n):
            row_t = float(t_values[idx])
            elapsed = (time.time() - start_time) * args.scale
            to_sleep = (row_t - t0) - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

            row = df.iloc[idx]

            # ATTITUDE - if quaternion exists send attitude (try convenience, fallback to constructor)
            if q0 and q1 and q2 and q3:
                try:
                    qw = safe_float(row[q0]); qx = safe_float(row[q1]); qy = safe_float(row[q2]); qz = safe_float(row[q3])
                    # compute rpy
                    ysqr = qy*qy
                    roll = math.atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + ysqr))
                    t2a = +2.0 * (qw * qy - qz * qx)
                    t2a = +1.0 if t2a > +1.0 else t2a
                    t2a = -1.0 if t2a < -1.0 else t2a
                    pitch = math.asin(t2a)
                    yaw_deg = quat_to_yaw_deg(qw, qx, qy, qz)
                    yaw = math.radians(yaw_deg)
                    try:
                        master.mav.attitude_send(int((row_t - t0) * 1000), float(roll), float(pitch), float(yaw), 0.0, 0.0, 0.0)
                    except Exception:
                        try:
                            msg = mavutil.mavlink.MAVLink_attitude_message(int((row_t - t0) * 1000),
                                                                           float(roll), float(pitch), float(yaw),
                                                                           0.0, 0.0, 0.0)
                            master.mav.send(msg)
                        except Exception:
                            pass
                except Exception:
                    pass

            # LOCAL_POSITION_NED
            if x_col and y_col and z_col:
                try:
                    x = safe_float(row[x_col]); y = safe_float(row[y_col]); z = safe_float(row[z_col])
                    vx = safe_float(row[vx_col]) if vx_col else 0.0
                    vy = safe_float(row[vy_col]) if vy_col else 0.0
                    vz = safe_float(row[vz_col]) if vz_col else 0.0
                    try:
                        master.mav.local_position_ned_send(int((row_t - t0) * 1000), x, y, z, vx, vy, vz)
                    except Exception:
                        try:
                            msg = mavutil.mavlink.MAVLink_local_position_ned_message(int((row_t - t0) * 1000), x, y, z, vx, vy, vz)
                            master.mav.send(msg)
                        except Exception:
                            pass
                except Exception:
                    pass

            # GLOBAL_POSITION_INT - robust send with the proper constructor signature
            if lat_col and lon_col and alt_col:
                try:
                    lat_i, lon_i, alt_mm = latlon_to_global_int(safe_float(row[lat_col]), safe_float(row[lon_col]), safe_float(row[alt_col]))
                except Exception:
                    lat_i, lon_i, alt_mm = 0, 0, 0

                # relative altitude: prefer CSV column, else use same as alt (mm)
                if rel_alt_col:
                    rel_alt_mm = safe_int(float(row[rel_alt_col]) * 1000.0, default=alt_mm)
                else:
                    rel_alt_mm = alt_mm

                vx_i = safe_int(float(row[vx_col]) * 100.0, default=0) if vx_col else 0
                vy_i = safe_int(float(row[vy_col]) * 100.0, default=0) if vy_col else 0
                vz_i = safe_int(float(row[vz_col]) * 100.0, default=0) if vz_col else 0

                # compute hdg from quaternion if present, else unknown
                hdg_to_send = 65535
                try:
                    if q0 and q1 and q2 and q3:
                        yaw_deg = quat_to_yaw_deg(float(row[q0]), float(row[q1]), float(row[q2]), float(row[q3]))
                        hdg_to_send = int(round(yaw_deg * 100.0)) % 36000
                except Exception:
                    hdg_to_send = 65535

                time_boot_ms = int((row_t - t0) * 1000)

                # Try convenience send (with hdg)
                try:
                    master.mav.global_position_int_send(time_boot_ms, lat_i, lon_i, alt_mm, vx_i, vy_i, vz_i, int(hdg_to_send))
                    # Note: some pymavlink versions expect the args in slightly different order.
                    # If this worked no exception will be raised.
                except TypeError as te:
                    # Fallback: construct local constructor that matches your pymavlink signature:
                    # we know from introspection your constructor expects:
                    # (self, time_boot_ms: int, lat: int, lon: int, alt: int, relative_alt: int, vx: int, vy: int, vz: int, hdg: int)
                    try:
                        msg = mavutil.mavlink.MAVLink_global_position_int_message(
                            time_boot_ms, lat_i, lon_i, alt_mm, rel_alt_mm, vx_i, vy_i, vz_i, int(hdg_to_send)
                        )
                        master.mav.send(msg)
                    except Exception as e2:
                        # final attempt: try convenience send without heading (7 args)
                        try:
                            master.mav.global_position_int_send(time_boot_ms, lat_i, lon_i, alt_mm, vx_i, vy_i, vz_i)
                        except Exception as e3:
                            print("ERROR: global_position_int send failed:", te, e2, e3)

                except Exception as e:
                    # other unexpected exception
                    print("WARNING: global_position_int raised unexpected exception:", e)

            # VFR_HUD (optional)
            if speed_col:
                try:
                    sp = safe_float(row[speed_col])
                    master.mav.vfr_hud_send(float(sp), 0.0, 0.0, 0.0, 0.0)
                except Exception:
                    pass

        # end row loop

        if args.loop:
            t0 = float(t_values[0])
            continue
        else:
            break

    print("Replay finished.")

if __name__ == '__main__':
    main()
