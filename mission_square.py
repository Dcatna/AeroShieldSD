# Run with: python3 mission_square.py [udpin://0.0.0.0:14540] (dont need to specify ip if ur using default)
import asyncio
import math
import subprocess, sys
from logger import Logger
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

DEFAULT_ADDR = "udpin://0.0.0.0:14540"

def mk_wp(lat, lon, rel_alt, speed, fly_through=True):
    """Create a mission waypoint with sane defaults for the newer MissionItem API."""
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=rel_alt,
        speed_m_s=speed,
        is_fly_through=fly_through,
        gimbal_pitch_deg=float("nan"),
        gimbal_yaw_deg=float("nan"),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0,
        camera_photo_interval_s=0.0,
        acceptance_radius_m=2.0,
        yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE
    )

def meters_to_latlon(d_north_m: float, d_east_m: float, ref_lat_deg: float): #gpt made these two functions 
    """Approx convert N/E meters -> lat/lon degrees around a given latitude."""
    d_lat = d_north_m / 111_111.0
    d_lon = d_east_m / (111_111.0 * math.cos(math.radians(ref_lat_deg)))
    return d_lat, d_lon

async def build_square(drone, rel_alt_m=30.0, side_m=40.0, speed=5.0):
    """Create a 4-waypoint square around current HOME position (returns to start)."""
    home = await drone.telemetry.home().__anext__()  # first sample
    lat0 = home.latitude_deg
    lon0 = home.longitude_deg

    # Offsets in meters (NESW loop starting and ending near home)
    # Waypoint order: north -> east -> south -> west (back close to home)
    corners_ne = [
        ( side_m,    0.0),   # north
        ( side_m,  side_m),  # east
        (   0.0 ,  side_m),  # south (relative)
        (   0.0 ,    0.0),   # back to start east-wise (near home)
    ]

    items = []
    for n, e in corners_ne:
        dlat, dlon = meters_to_latlon(n, e, lat0)
        lat = lat0 + dlat
        lon = lon0 + dlon
        items.append(mk_wp(lat, lon, rel_alt_m, speed, fly_through=True))

    # Optionally add a final point right on home (helps some planners “close” the loop)
    items.append(mk_wp(lat0, lon0, rel_alt_m, speed, fly_through=True))

    return MissionPlan(items)

async def run_mission(connection_url):
    drone = System()

    await drone.connect(system_address=connection_url)

    mav_logger = Logger(drone, "sqaure_mission")
    await mav_logger.start()

    print("Waiting for PX4...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    print("Waiting for home position...") #wait to be home
    home = await drone.telemetry.home().__anext__()
    print(f"Home: {home.latitude_deg:.7f}, {home.longitude_deg:.7f}")

    try: # make sure its a fresh mission
        await drone.mission.clear_mission()
    except Exception:
        pass

    plan = await build_square(drone, rel_alt_m=30.0, side_m=40.0, speed=5.0) # create and upload mission
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(plan)
    print(f"Uploaded mission with {len(plan.mission_items)} waypoints.")

    await drone.action.arm() #arm and start the drone !
    await drone.mission.start_mission()
    print("Mission started.")

    async for prog in drone.mission.mission_progress(): #watch progress
        print(f"Mission progress: {prog.current}/{prog.total}")
        if prog.total > 0 and prog.current >= prog.total:
            print("Mission finished.")
            break

    print("Waiting for landing/disarm...") # land and disarm
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break

    try:
        await drone.action.disarm() #just make sure again lol
    except Exception:
        pass

    await mav_logger.stop()
    print("Done.")

if __name__ == "__main__":
    addr = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ADDR
    if addr.startswith("udpin://:"):
        port = addr.split(":")[-1]
        addr = f"udpin://0.0.0.0:{port}"

    
    asyncio.run(run_mission(addr))
