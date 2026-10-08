import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

ALT = 8.0          # takeoff altitude (m)
FWD_SPEED = 2.0    # forward speed (m/s)
FWD_TIME = 8       # seconds flying forward (~16 m)
DETECT_TIME = 8    # seconds hovering to detect

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("Connected"); break

    print("Waiting for position lock...")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            print("Ready"); break

    print("Arming"); await drone.action.arm()
    print("Takeoff"); await drone.action.set_takeoff_altitude(ALT)
    await drone.action.takeoff()
    await asyncio.sleep(10)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e._result.result}")
        await drone.action.land(); return

    print("Flying forward")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(FWD_SPEED,0,0,0))
    await asyncio.sleep(FWD_TIME)

    print("Hovering to detect")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    await asyncio.sleep(DETECT_TIME)

    await drone.offboard.stop()
    print("Landing")
    await drone.action.land()
    await asyncio.sleep(12)
    print("Done")

if __name__ == "__main__":
    asyncio.run(run())
