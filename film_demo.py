import asyncio, math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

ALT = 4.0           # go up a little (m)
FWD_DIST = 10.0     # slow forward distance (m) - tune to pass all the cars
STOP_THRESH = 0.12  # m/s -> fully stopped

class V:
    sp = 99.0

async def vel_reader(drone, v):
    async for vel in drone.telemetry.velocity_ned():
        v.sp = math.sqrt(vel.north_m_s**2 + vel.east_m_s**2)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for s in drone.core.connection_state():
        if s.is_connected: break
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok: break

    print("Up a little")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(ALT)
    await drone.action.takeoff()
    await asyncio.sleep(7)

    async for pv in drone.telemetry.position_velocity_ned():
        sn, se, sd = pv.position.north_m, pv.position.east_m, pv.position.down_m
        break
    async for att in drone.telemetry.attitude_euler():
        yaw = att.yaw_deg
        break
    yr = math.radians(yaw)
    tn = sn + FWD_DIST*math.cos(yr)
    te = se + FWD_DIST*math.sin(yr)

    v = V()
    asyncio.ensure_future(vel_reader(drone, v))

    await drone.offboard.set_position_ned(PositionNedYaw(sn, se, sd, yaw))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e._result.result}"); await drone.action.land(); return

    print("Forward slowly (detecting)...")
    t = 0.0
    while t < 30.0:
        await drone.offboard.set_position_ned(PositionNedYaw(tn, te, sd, yaw))
        await asyncio.sleep(0.1); t += 0.1
        if t > 4.0 and v.sp < STOP_THRESH:
            break

    print("Fully stopped, settling")
    t = 0.0
    while t < 4.0:
        await drone.offboard.set_position_ned(PositionNedYaw(tn, te, sd, yaw))
        await asyncio.sleep(0.1); t += 0.1

    await drone.offboard.stop()
    print("Soft vertical landing")
    await drone.action.land()
    async for armed in drone.telemetry.armed():
        if not armed:
            print("Landed, motors off"); break

if __name__ == "__main__":
    asyncio.run(run())
