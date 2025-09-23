import asyncio
from mavsdk import System

async def main():
    print("Waiting for connection")

    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    async for state in drone.core.connection_state():
        if( state.is_connected):
            print(f"Connected to PX4, State: {state}")
            break

    print("Subbing to heartbeat, flight mode, position")

    async def hb_printer():
        async for hb in drone.telemetry.health():
            if hb.is_global_position_ok and hb.is_home_position_ok:
                print("Heartbeat: sensors healthy, GPS & home OK")
                break

    async def mode_printer():
        async for mode in drone.telemetry.flight_mode():
            print(f"Flight mode: {mode.name if hasattr(mode,'name') else mode}")
            break

    async def position_printer():
        # just print a few can log all later if wanted
        count = 0
        async for pos in drone.telemetry.position():
            print(f"Lat {pos.latitude_deg:.7f}, Lon {pos.longitude_deg:.7f}, Alt {pos.relative_altitude_m:.1f} m")
            count += 1
            if count >= 5:
                break

    try:
        await asyncio.wait_for(asyncio.gather(hb_printer(), mode_printer(), position_printer()), timeout=10) #sub
    except asyncio.TimeoutError:
        print("Timed out waiting for telemetry (is PX4 SITL running?)")
    

if __name__ == "__main__":
    asyncio.run(main())