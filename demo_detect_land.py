import asyncio, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

ALT = 6.0               # vertical takeoff height (m)
FWD_SPEED = 1.5         # horizontal speed (m/s)
MAX_FWD_TIME = 25       # safety cap (s)
EXTRA_AFTER_DETECT = 4  # keep flying forward AFTER detection to get over the car
BRAKE_TIME = 4          # full stop before landing -> straight-down descent

class State:
    detected = False

class Listener(Node):
    def __init__(self, state):
        super().__init__('demo_listener')
        self.state = state
        self.create_subscription(Bool, '/target_detected', self.cb, 10)
    def cb(self, msg):
        if msg.data:
            self.state.detected = True

def ros_spin(state):
    rclpy.init()
    rclpy.spin(Listener(state))

async def forward(drone, seconds, speed):
    t = 0.0
    while t < seconds:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(speed,0,0,0))
        await asyncio.sleep(0.2); t += 0.2

async def hold(drone, seconds):
    t = 0.0
    while t < seconds:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
        await asyncio.sleep(0.2); t += 0.2

async def run():
    state = State()
    threading.Thread(target=ros_spin, args=(state,), daemon=True).start()

    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for s in drone.core.connection_state():
        if s.is_connected: break
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok: break

    print("Arming / vertical takeoff")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(ALT)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e._result.result}")
        await drone.action.land(); return

    print("Horizontal forward, watching for a car...")
    t = 0.0
    while t < MAX_FWD_TIME and not state.detected:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(FWD_SPEED,0,0,0))
        await asyncio.sleep(0.2); t += 0.2

    if state.detected:
        print("CAR DETECTED - flying over it")
        await forward(drone, EXTRA_AFTER_DETECT, FWD_SPEED)
    else:
        print("No detection - landing anyway")

    print("Full stop")
    await hold(drone, BRAKE_TIME)

    await drone.offboard.stop()
    print("Vertical landing")
    await drone.action.land()
    async for armed in drone.telemetry.armed():
        if not armed:
            print("Landed and disarmed"); break

if __name__ == "__main__":
    asyncio.run(run())
