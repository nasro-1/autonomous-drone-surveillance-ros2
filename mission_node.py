#!/usr/bin/env python3
"""
mission_node.py — Autonomous Payload Mission Node
===================================================
Hooks into the existing PX4-ROS2-Gazebo-YOLOv8 repo.

Subscribes to:
  /target_detected  (std_msgs/Bool)   — published by uav_camera_det.py (patched)
  /target_bbox      (std_msgs/String) — "cx,cy,w,h" in pixel space
  /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry) — drone pose

Mission state machine:
  IDLE → TAKEOFF → WAYPOINT_NAV → SEARCH →
  TARGET_LOCKED → APPROACH → PAYLOAD_RELEASE → RTH → LANDED

Author: Your name
"""

import asyncio
import math
import time
import json
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ── mission parameters ────────────────────────────────────────────────────────
SEARCH_ALTITUDE   = 20.0   # metres AGL during waypoint patrol
APPROACH_ALTITUDE = 8.0    # metres AGL when descending on target
LOCK_THRESHOLD    = 60     # consecutive frames with detection before commit
LOCK_CENTRE_TOL   = 80     # pixels: bounding-box centre must be within this
PAYLOAD_HOLD_TIME = 2.0    # seconds to "hold" at release point
LOG_PATH          = os.path.expanduser("~/mission_logs")

# Waypoints: lat, lon, alt (Gazebo Garden default origin offsets in metres)
# These are converted to NED offsets from home; adjust to your world
PATROL_WAYPOINTS_NED = [          # (North_m, East_m, Down_m)
    (30.0,   0.0, -SEARCH_ALTITUDE),
    (30.0,  30.0, -SEARCH_ALTITUDE),
    ( 0.0,  30.0, -SEARCH_ALTITUDE),
    ( 0.0,   0.0, -SEARCH_ALTITUDE),
]

# ── state machine ─────────────────────────────────────────────────────────────
class MissionState:
    IDLE            = "IDLE"
    TAKEOFF         = "TAKEOFF"
    WAYPOINT_NAV    = "WAYPOINT_NAV"
    SEARCH          = "SEARCH"
    TARGET_LOCKED   = "TARGET_LOCKED"
    APPROACH        = "APPROACH"
    PAYLOAD_RELEASE = "PAYLOAD_RELEASE"
    RTH             = "RTH"
    LANDED          = "LANDED"


class PayloadMissionNode(Node):
    """ROS2 node that drives the full autonomous payload mission."""

    def __init__(self):
        super().__init__("payload_mission_node")

        # ── subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Bool,   "/target_detected", self._cb_detected, 10)
        self.create_subscription(String, "/target_bbox",     self._cb_bbox,     10)

        # ── publishers ──────────────────────────────────────────────────────
        self.state_pub    = self.create_publisher(String, "/mission/state",   10)
        self.mission_pub  = self.create_publisher(String, "/mission/log",     10)

        # ── internal state ───────────────────────────────────────────────────
        self.state        = MissionState.IDLE
        self.detected     = False
        self.bbox         = None          # (cx, cy, w, h) in pixels
        self.lock_counter = 0
        self.image_w      = 640
        self.image_h      = 480
        self.drone_ned    = (0.0, 0.0, 0.0)   # last known NED position
        self.target_gps   = None               # estimated GPS of target
        self.mission_log  = []

        os.makedirs(LOG_PATH, exist_ok=True)
        self.log_file = os.path.join(
            LOG_PATH,
            f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )

        self.get_logger().info("PayloadMissionNode initialised. Starting mission loop.")

        # Run async MAVSDK mission in a background thread
        import threading
        t = threading.Thread(target=self._run_async_loop, daemon=True)
        t.start()

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _cb_detected(self, msg: Bool):
        self.detected = msg.data

    def _cb_bbox(self, msg: String):
        try:
            parts = [float(x) for x in msg.data.split(",")]
            if len(parts) == 4:
                self.bbox = tuple(parts)   # cx, cy, w, h
        except Exception:
            pass

    # ── helpers ───────────────────────────────────────────────────────────────
    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.get_logger().info(f"[STATE] {self.state}")

    def _log(self, event: str, data: dict = None):
        entry = {
            "timestamp": datetime.now().isoformat(),
            "state": self.state,
            "event": event,
        }
        if data:
            entry.update(data)
        self.mission_log.append(entry)
        msg = String()
        msg.data = json.dumps(entry)
        self.mission_pub.publish(msg)
        self.get_logger().info(f"[LOG] {event}")

    def _estimate_target_gps(self):
        """
        Simple pixel-to-world estimate.
        Uses pinhole projection: offset from image centre gives angular offset,
        multiplied by altitude gives ground distance.
        Returns (north_offset_m, east_offset_m) relative to current drone pos.
        """
        if self.bbox is None:
            return None
        cx, cy, _, _ = self.bbox
        alt = abs(self.drone_ned[2])          # altitude in metres (NED down is negative)
        fov_h = math.radians(90.0)            # horizontal FOV of IMX214 in SDF
        fov_v = math.radians(67.5)            # vertical FOV

        # pixel offsets from centre
        dx_px = cx - self.image_w / 2.0
        dy_px = cy - self.image_h / 2.0

        # angular offsets
        angle_x = (dx_px / (self.image_w / 2.0)) * (fov_h / 2.0)
        angle_y = (dy_px / (self.image_h / 2.0)) * (fov_v / 2.0)

        # ground offsets (East, North)
        east_offset  =  math.tan(angle_x) * alt
        north_offset = -math.tan(angle_y) * alt   # image Y grows downward

        return north_offset, east_offset

    def _target_centered(self):
        """True if bounding box centre is within tolerance of image centre."""
        if self.bbox is None:
            return False
        cx, cy, _, _ = self.bbox
        dx = abs(cx - self.image_w / 2.0)
        dy = abs(cy - self.image_h / 2.0)
        return dx < LOCK_CENTRE_TOL and dy < LOCK_CENTRE_TOL

    def _save_log(self):
        with open(self.log_file, "w", newline="") as f:
            if self.mission_log:
                writer = csv.DictWriter(f, fieldnames=self.mission_log[0].keys())
                writer.writeheader()
                writer.writerows(self.mission_log)
        self.get_logger().info(f"Mission log saved → {self.log_file}")

    # ── async MAVSDK mission ──────────────────────────────────────────────────
    def _run_async_loop(self):
        asyncio.run(self._mission_loop())

    async def _mission_loop(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.get_logger().info("Waiting for drone connection…")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break

        # ── TAKEOFF ──────────────────────────────────────────────────────────
        self.state = MissionState.TAKEOFF
        self._publish_state()
        self._log("arm_and_takeoff")

        await drone.action.arm()
        await drone.action.takeoff()
        await asyncio.sleep(6)                   # wait for stable hover

        # ── WAYPOINT NAV ─────────────────────────────────────────────────────
        self.state = MissionState.WAYPOINT_NAV
        self._publish_state()
        self._log("waypoint_nav_start")

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        try:
            await drone.offboard.start()
        except OffboardError as e:
            self.get_logger().error(f"Offboard start failed: {e}")
            return

        for idx, (n, e, d) in enumerate(PATROL_WAYPOINTS_NED):
            self._log(f"heading_to_waypoint_{idx}", {"N": n, "E": e, "D": d})
            await self._fly_to_ned(drone, n, e, d)
            await asyncio.sleep(1)

            # check for target during transit
            if self.detected:
                self._log("target_spotted_during_transit")
                break

        # ── SEARCH ───────────────────────────────────────────────────────────
        self.state = MissionState.SEARCH
        self._publish_state()
        self._log("search_start")

        search_done = False
        search_timeout = time.time() + 60          # 60-second search window
        while not search_done and time.time() < search_timeout:
            await asyncio.sleep(0.1)
            if self.detected and self._target_centered():
                self.lock_counter += 1
            else:
                self.lock_counter = max(0, self.lock_counter - 1)

            if self.lock_counter >= LOCK_THRESHOLD:
                search_done = True

        if not search_done:
            self._log("search_timeout_no_target")
            await self._rth(drone)
            return

        # ── TARGET LOCKED ────────────────────────────────────────────────────
        self.state = MissionState.TARGET_LOCKED
        self._publish_state()
        offset = self._estimate_target_gps()
        self._log("target_locked", {
            "lock_frames": self.lock_counter,
            "offset_N": offset[0] if offset else None,
            "offset_E": offset[1] if offset else None,
        })

        # ── APPROACH ─────────────────────────────────────────────────────────
        self.state = MissionState.APPROACH
        self._publish_state()

        if offset:
            n_cur, e_cur, _ = self.drone_ned
            target_n = n_cur + offset[0]
            target_e = e_cur + offset[1]
            self._log("approach_start", {"target_N": target_n, "target_E": target_e})
            # Descend toward target
            await self._fly_to_ned(drone, target_n, target_e, -APPROACH_ALTITUDE)
        else:
            # Hover descend straight down
            self._log("approach_descend_in_place")
            await self._fly_to_ned(drone, *self.drone_ned[:2], -APPROACH_ALTITUDE)

        await asyncio.sleep(2)

        # ── PAYLOAD RELEASE ──────────────────────────────────────────────────
        self.state = MissionState.PAYLOAD_RELEASE
        self._publish_state()
        self._log("payload_release", {
            "drone_N": self.drone_ned[0],
            "drone_E": self.drone_ned[1],
            "altitude": APPROACH_ALTITUDE,
            "timestamp": datetime.now().isoformat(),
        })

        # Simulated payload: hover in place for PAYLOAD_HOLD_TIME seconds
        # In a real system you would send a GPIO/MAVLink command here.
        await asyncio.sleep(PAYLOAD_HOLD_TIME)
        self._log("payload_released_success")

        # ── RTH ───────────────────────────────────────────────────────────────
        await self._rth(drone)

    async def _fly_to_ned(self, drone, n: float, e: float, d: float,
                          speed: float = 5.0, tol: float = 1.5):
        """
        Fly to NED position at given speed using offboard velocity control.
        Updates self.drone_ned as it moves.
        """
        async for odom in drone.telemetry.odometry():
            n_cur = odom.position_body.x_m
            e_cur = odom.position_body.y_m
            d_cur = -odom.position_body.z_m     # NED down is positive
            self.drone_ned = (n_cur, e_cur, d_cur)
            break

        while True:
            async for odom in drone.telemetry.odometry():
                n_cur = odom.position_body.x_m
                e_cur = odom.position_body.y_m
                d_cur = -odom.position_body.z_m
                self.drone_ned = (n_cur, e_cur, d_cur)
                break

            dn = n - n_cur
            de = e - e_cur
            dd = d - d_cur
            dist = math.sqrt(dn**2 + de**2 + dd**2)

            if dist < tol:
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                break

            scale = min(speed, dist) / dist
            vn = dn * scale
            ve = de * scale
            vd = dd * scale      # positive = down in NED

            yaw = math.degrees(math.atan2(de, dn)) % 360

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, yaw)
            )
            await asyncio.sleep(0.1)

    async def _rth(self, drone):
        self.state = MissionState.RTH
        self._publish_state()
        self._log("rth_start")

        try:
            await drone.offboard.stop()
        except Exception:
            pass

        await drone.action.return_to_launch()

        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break
            await asyncio.sleep(1)

        self.state = MissionState.LANDED
        self._publish_state()
        self._log("landed")
        self._save_log()


def main(args=None):
    rclpy.init(args=args)
    node = PayloadMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
