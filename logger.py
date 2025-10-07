
# run command:
#   python3 mavsdk_logger.py [udpin://0.0.0.0:14540]
#
# listens on udpin://0.0.0.0:14540 (SITL default). The script:
#  - connects to PX4 via MAVSDK
#  - subscribes to health, flight mode, position, battery, and mission progress
#  - writes a timestamped text log (appendable) and prints to console
#  - exits cleanly after a short sampling period (adjustable)

import asyncio
import sys
import datetime
from mavsdk import System

DEFAULT_ADDR = "udpin://0.0.0.0:14540"
SAMPLE_DURATION_SECS = 30   # keep for now just to have a default run time

def open_logfile():
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    fname = f"drone_trace_{stamp}.txt"
    f = open(fname, "a", encoding="utf-8")
    return f, fname

def log_line(f, s):
    ts = datetime.datetime.utcnow().isoformat(timespec="milliseconds") + "Z"
    line = f"[{ts}] {s}"
    print(line)
    f.write(line + "\n")
    f.flush()

async def collect(connection_url):
    f, fname = open_logfile()
    log_line(f, f"Starting mavsdk_logger. Connection: {connection_url}. Output file: {fname}")

    drone = System()
    await drone.connect(system_address=connection_url)

    # wait for connection
    connected = False
    try:
        async for state in drone.core.connection_state():
            if state.is_connected:
                log_line(f, f"Connected to PX4. ConnectionState: {state}")
                connected = True
                break
    except Exception as e:
        log_line(f, f"Error while waiting for connection: {e}")
        f.close()
        return

    if not connected:
        log_line(f, "Failed to connect to PX4.")
        f.close()
        return

    # record start time 
    end_time = datetime.datetime.utcnow() + datetime.timedelta(seconds=SAMPLE_DURATION_SECS)

    async def health_task():
        async for hb in drone.telemetry.health():
            log_line(f, f"HEALTH gps_ok={hb.is_global_position_ok} home_ok={hb.is_home_position_ok} local_ok={hb.is_local_position_ok}")
            # small sleep so stream isnt spammy
            await asyncio.sleep(0.2)
            if datetime.datetime.utcnow() >= end_time:
                break

    async def mode_task():
        async for mode in drone.telemetry.flight_mode():
            mode_str = mode.name if hasattr(mode, "name") else str(mode)
            log_line(f, f"FLIGHT_MODE {mode_str}")
            await asyncio.sleep(0.2)
            if datetime.datetime.utcnow() >= end_time:
                break

    async def pos_task():
        async for pos in drone.telemetry.position():
            lat = pos.latitude_deg
            lon = pos.longitude_deg
            rel_alt = pos.relative_altitude_m if pos.relative_altitude_m is not None else float('nan')
            log_line(f, f"POSITION lat={lat:.7f} lon={lon:.7f} rel_alt_m={rel_alt:.2f}")
            await asyncio.sleep(0.2)
            if datetime.datetime.utcnow() >= end_time:
                break

    async def bat_task():
        async for bat in drone.telemetry.battery():
            percent = bat.remaining_percent * 100 if bat.remaining_percent is not None else float('nan')
            volt = getattr(bat, "voltage_v", None)
            volt_str = f"{volt:.2f}V" if volt is not None else "unknownV"
            log_line(f, f"BATTERY {percent:.1f}% {volt_str}")
            await asyncio.sleep(1.0)
            if datetime.datetime.utcnow() >= end_time:
                break

    async def mission_task():
        async for prog in drone.mission.mission_progress():
            log_line(f, f"MISSION_PROGRESS current={prog.current} total={prog.total}")
            await asyncio.sleep(0.5)
            if datetime.datetime.utcnow() >= end_time:
                break

    # run all tasks concurrently
    try:
        await asyncio.wait_for(
            asyncio.gather(health_task(), mode_task(), pos_task(), bat_task(), mission_task()),
            timeout=SAMPLE_DURATION_SECS + 5
        )
    except asyncio.TimeoutError:
        log_line(f, "Sampling duration reached or timeout.")

    # system info
    try:
        info = await drone.info.get_version()
        log_line(f, f"PX4_VERSION {info.flight_sw_major}.{info.flight_sw_minor}.{info.flight_sw_patch} ({info.flight_sw_git_hash})")
    except Exception as e:
        log_line(f, f"Could not fetch PX4 version info: {e}")

    await drone.close()
    log_line(f, "Closed connection and finished logging.")
    f.close() #just saves to a txt with the time stamp (kinda ugly)
    print(f"Log saved to: {fname}")

if __name__ == "__main__":
    addr = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ADDR

    if addr.startswith("udp://:"): # for passing in ur own ip (so can do udp://:14540 or change 14540 for anyhtign u want)
        port = addr.split(":")[-1]
        addr = f"udpin://0.0.0.0:{port}"
    asyncio.run(collect(addr))
