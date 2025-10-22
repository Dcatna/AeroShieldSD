#!/usr/bin/env python3
"""
hil_replay_test.py
Lightweight HIL-style replayer sending HIL_GPS + HIL_SENSOR from a merged CSV.
Usage:
  python3 hil_replay_test.py merged_replay.cleaned.csv --target udp:127.0.0.1:14580 --rows 200 --scale 1.0

Notes:
 - Adjust column names below if your merged CSV uses different keys.
 - This script does best-effort conversions; improve mappings with actual CSV column names.
"""
import argparse, time, math
import pandas as pd
from pymavlink import mavutil

# --- Mapping: change these keys if your merged CSV uses other names
COL_TIME = None  # if None the script uses the first column
COL_LAT = '02_41_26.vehicle_gps_position::latitude_deg'
COL_LON = '02_41_26.vehicle_gps_position::longitude_deg'
COL_ALT = '02_41_26.vehicle_gps_position::altitude_msl_m'      # meters
COL_VX  = '02_41_26.vehicle_local_position::vx'                # m/s (N)
COL_VY  = '02_41_26.vehicle_local_position::vy'                # m/s (E)
COL_VZ  = '02_41_26.vehicle_local_position::vz'                # m/s (D positive down)
# IMU columns (best-effort names)
COL_AX = '02_41_26.vehicle_imu::xacc'     # m/s^2
COL_AY = '02_41_26.vehicle_imu::yacc'
COL_AZ = '02_41_26.vehicle_imu::zacc'
COL_GX = '02_41_26.vehicle_imu::xgyro'    # rad/s
COL_GY = '02_41_26.vehicle_imu::ygyro'
COL_GZ = '02_41_26.vehicle_imu::zgyro'
COL_P_BARO = '02_41_26.vehicle_air_data::baro_pres_pa'  # try to use baro if present

def safe_get(row, key, default=0.0):
    return float(row[key]) if (key in row and not pd.isna(row[key])) else default

def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv', help='merged csv file')
    p.add_argument('--target', default='udp:127.0.0.1:14580', help='mavlink target (use PX4 onboard port)')
    p.add_argument('--rows', type=int, default=200, help='max rows to send (for quick test)')
    p.add_argument('--scale', type=float, default=1.0, help='time scale: 1.0 = real row spacing')
    p.add_argument('--dry', action='store_true', help='dry-run: do not send, only preview')
    args = p.parse_args()

    df = pd.read_csv(args.csv, low_memory=False)
    if COL_TIME is None:
        time_col = df.columns[0]
    else:
        time_col = COL_TIME

    # quick preview
    print("Preview first 8 rows (time, lat, lon, alt_m, vx, vy, vz):")
    for i, r in df.head(8).iterrows():
        print(r.get(time_col), r.get(COL_LAT), r.get(COL_LON), r.get(COL_ALT),
              r.get(COL_VX), r.get(COL_VY), r.get(COL_VZ))

    if args.dry:
        print("Dry run - exiting")
        return

    print("Connecting to", args.target)
    master = mavutil.mavlink_connection(args.target, source_system=250)
    print("Waiting for heartbeat from system (connected).")
    master.wait_heartbeat(timeout=10)
    print("Heartbeat received. Starting replay...")

    prev_t = None
    sent = 0
    for i, row in df.iterrows():
        if sent >= args.rows:
            break

        t_s = float(row[time_col])
        if prev_t is None:
            dt = 0.0
        else:
            dt = (t_s - prev_t) * args.scale
            if dt < 0:
                dt = 0.0
        prev_t = t_s

        # build HIL_GPS fields
        lat = safe_get(row, COL_LAT, 0.0)
        lon = safe_get(row, COL_LON, 0.0)
        alt_m = safe_get(row, COL_ALT, 0.0)
        vx = safe_get(row, COL_VX, 0.0)
        vy = safe_get(row, COL_VY, 0.0)
        vz = safe_get(row, COL_VZ, 0.0)

        time_usec = int(t_s * 1e6)
        lat_i = int(lat * 1e7)
        lon_i = int(lon * 1e7)
        alt_mm = int(alt_m * 1000.0)

        # velocities in cm/s (MAV HIL expects cm/s)
        vn = int(round(vx * 100.0))
        ve = int(round(vy * 100.0))
        vd = int(round(vz * 100.0))
        speed = int(round(math.hypot(vx, vy) * 100.0))

        # send HIL_GPS (time_usec, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, fix_type, satellites_visible)
        try:
            master.mav.hil_gps_send(time_usec, lat_i, lon_i, alt_mm,
                                    65535, 65535, speed, vn, ve, vd, 65535, 3, 10)
        except Exception as e:
            print("HIL_GPS send failed:", e)

        # HIL_SENSOR: choose sensible defaults if column missing
        ax = safe_get(row, COL_AX, 0.0)
        ay = safe_get(row, COL_AY, 0.0)
        az = safe_get(row, COL_AZ, 0.0)
        gx = safe_get(row, COL_GX, 0.0)
        gy = safe_get(row, COL_GY, 0.0)
        gz = safe_get(row, COL_GZ, 0.0)

        abs_pressure = safe_get(row, COL_P_BARO, 101325.0)    # Pa
        diff_pressure = 0.0
        pressure_alt = int(alt_m * 1000.0)
        temperature = 20.0

        try:
            master.mav.hil_sensor_send(time_usec,
                                      ax, ay, az,
                                      gx, gy, gz,
                                      0.0, 0.0, 0.0,
                                      abs_pressure, diff_pressure, pressure_alt,
                                      temperature,
                                      0xFFFFFFFF)
        except Exception as e:
            print("HIL_SENSOR send failed:", e)

        # small throttle so we don't saturate network
        time.sleep(max(0.0, dt))

        sent += 1

    print("Replay finished. Sent rows:", sent)
    master.close()

if __name__ == '__main__':
    main()
