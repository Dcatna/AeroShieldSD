
import asyncio
import datetime
import sys
from mavsdk import System

DEFAULT_ADDR = "udpin://0.0.0.0:14540"

def _open_logfile(mission_name: str):
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    fname = f"drone_trace_{stamp}_{mission_name}.txt"
    fh = open(fname, "a", encoding="utf-8")
    return fh, fname

def _log_line(fh, msg: str):
    ts = datetime.datetime.utcnow().isoformat(timespec="milliseconds") + "Z"
    line = f"[{ts}] {msg}"
    print(line)
    fh.write(line + "\n")
    fh.flush()

class Logger:
    """Defines the functions for starting stopping and creating logs"""
    def __init__(self, drone: System, mission_name: str = "mission"):
        self._drone = drone
        self._mission_name = mission_name or "mission"
        self._fh = None
        self._fname = None
        self._stop = asyncio.Event()
        self._tasks = []

    async def start(self):
        """Start telemetry subscriptions and open log file."""
        self._fh, self._fname = _open_logfile(self._mission_name)
        _log_line(self._fh, f"LOGGER start mission={self._mission_name}")

        # try to print version
        try:
            info = await self._drone.info.get_version()
            _log_line(self._fh, f"PX4_VERSION {info.flight_sw_major}.{info.flight_sw_minor}.{info.flight_sw_patch} ({info.flight_sw_git_hash})")
        except Exception as e:
            _log_line(self._fh, f"PX4_VERSION error: {e}")

        self._tasks = [
            asyncio.create_task(self._log_health()),
            asyncio.create_task(self._log_mode()),
            asyncio.create_task(self._log_position()),
            asyncio.create_task(self._log_battery()),
            asyncio.create_task(self._log_mission_progress()),
        ]

    async def stop(self):
        """Signal tasks to stop, await them, and close the file."""
        if self._fh is None:
            return
        self._stop.set()
        for t in self._tasks: #let tasks finish nicely
            try:
                await t
            except asyncio.CancelledError:
                pass
            except Exception as e:
                _log_line(self._fh, f"TASK error: {e}")
        _log_line(self._fh, "LOGGER stop")
        self._fh.close()
        print(f"Log saved to: {self._fname}")

    async def _log_health(self): # actual log functions 
        async for hb in self._drone.telemetry.health():
            _log_line(self._fh, f"HEALTH gps_ok={hb.is_global_position_ok} home_ok={hb.is_home_position_ok} local_ok={hb.is_local_position_ok}")
            if self._stop.is_set(): break
            await asyncio.sleep(0.2)

    async def _log_mode(self):
        async for mode in self._drone.telemetry.flight_mode():
            name = getattr(mode, "name", str(mode))
            _log_line(self._fh, f"FLIGHT_MODE {name}")
            if self._stop.is_set(): break
            await asyncio.sleep(0.2)

    async def _log_position(self):
        async for pos in self._drone.telemetry.position():
            ra = pos.relative_altitude_m if pos.relative_altitude_m is not None else float("nan")
            _log_line(self._fh, f"POSITION lat={pos.latitude_deg:.7f} lon={pos.longitude_deg:.7f} rel_alt_m={ra:.2f}")
            if self._stop.is_set(): break
            await asyncio.sleep(0.2)

    async def _log_battery(self):
        async for bat in self._drone.telemetry.battery():
            pct = bat.remaining_percent * 100 if bat.remaining_percent is not None else float("nan")
            volt = getattr(bat, "voltage_v", None)
            vstr = f"{volt:.2f}V" if volt is not None else "unknownV"
            _log_line(self._fh, f"BATTERY {pct:.1f}% {vstr}")
            if self._stop.is_set(): break
            await asyncio.sleep(1.0)

    async def _log_mission_progress(self):
        async for prog in self._drone.mission.mission_progress():
            _log_line(self._fh, f"MISSION_PROGRESS current={prog.current} total={prog.total}")
            if self._stop.is_set(): break
            await asyncio.sleep(0.3)
