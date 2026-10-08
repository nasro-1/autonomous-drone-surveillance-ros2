import asyncio
from mavsdk import System

HOVER_ALT = 8.0      # meters
FLY_SECONDS = 20     # how long to stay up and detect

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Waiting for position lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Ready to fly")
            break

    print("Arming")
    await drone.action.arm()

    print(f"Taking off to {HOVER_ALT} m")
    await drone.action.set_takeoff_altitude(HOVER_ALT)
    await drone.action.takeoff()

    print(f"Hovering {FLY_SECONDS}s - YOLO is detecting now")
    await asyncio.sleep(FLY_SECONDS)

    print("Landing")
    await drone.action.land()
    await asyncio.sleep(12)
    print("Done")

if __name__ == "__main__":
    asyncio.run(run())
