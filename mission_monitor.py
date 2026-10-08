#!/usr/bin/env python3
"""
mission_monitor.py — Live Mission Dashboard
============================================
Run in a separate terminal to see the full mission state in real time.

Subscribes to:
  /mission/state        — current state machine state
  /mission/log          — JSON event log entries
  /target_detected      — bool
  /target_bbox          — "cx,cy,w,h"
  /target_confidence    — float

Usage:
  python mission_monitor.py
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

# ANSI colours
R  = "\033[91m"
G  = "\033[92m"
Y  = "\033[93m"
B  = "\033[94m"
M  = "\033[95m"
C  = "\033[96m"
W  = "\033[97m"
DIM = "\033[2m"
RST = "\033[0m"
BOLD = "\033[1m"

STATE_COLORS = {
    "IDLE":            DIM,
    "TAKEOFF":         C,
    "WAYPOINT_NAV":    B,
    "SEARCH":          Y,
    "TARGET_LOCKED":   M,
    "APPROACH":        R,
    "PAYLOAD_RELEASE": R + BOLD,
    "RTH":             G,
    "LANDED":          G + BOLD,
}


class MissionMonitor(Node):
    def __init__(self):
        super().__init__("mission_monitor")

        self.state      = "IDLE"
        self.detected   = False
        self.bbox       = "—"
        self.confidence = 0.0
        self.events     = []

        self.create_subscription(String,  "/mission/state",       self._cb_state,  10)
        self.create_subscription(String,  "/mission/log",         self._cb_log,    10)
        self.create_subscription(Bool,    "/target_detected",     self._cb_det,    10)
        self.create_subscription(String,  "/target_bbox",         self._cb_bbox,   10)
        self.create_subscription(Float32, "/target_confidence",   self._cb_conf,   10)

        self.create_timer(0.5, self._render)

    def _cb_state(self, msg):  self.state = msg.data
    def _cb_det(self,   msg):  self.detected = msg.data
    def _cb_bbox(self,  msg):  self.bbox = msg.data
    def _cb_conf(self,  msg):  self.confidence = msg.data

    def _cb_log(self, msg):
        try:
            entry = json.loads(msg.data)
            self.events.append(entry)
            if len(self.events) > 12:
                self.events.pop(0)
        except Exception:
            pass

    def _render(self):
        print("\033[2J\033[H", end="")   # clear screen
        print(f"{BOLD}{'═'*60}{RST}")
        print(f"{BOLD}  🎯 UAV PAYLOAD MISSION — LIVE MONITOR{RST}")
        print(f"{BOLD}{'═'*60}{RST}")

        color = STATE_COLORS.get(self.state, W)
        print(f"\n  State  : {color}{BOLD}{self.state:20s}{RST}")

        det_str = f"{G}YES ✓{RST}" if self.detected else f"{DIM}NO{RST}"
        print(f"  Target : {det_str}")

        if self.detected:
            print(f"  BBox   : {Y}{self.bbox}{RST}")
            print(f"  Conf   : {Y}{self.confidence:.2f}{RST}")
        else:
            print(f"  BBox   : {DIM}—{RST}")
            print(f"  Conf   : {DIM}—{RST}")

        print(f"\n  {DIM}{'─'*56}{RST}")
        print(f"  {BOLD}Event Log{RST}")
        print(f"  {DIM}{'─'*56}{RST}")

        for e in self.events[-8:]:
            ts = e.get("timestamp","")[-12:]
            ev = e.get("event", "?")
            st = e.get("state", "")
            data_keys = [k for k in e if k not in ("timestamp","event","state")]
            extras = "  ".join(f"{k}={e[k]}" for k in data_keys[:3])
            print(f"  {DIM}{ts}{RST}  {C}{st:<18s}{RST}  {W}{ev}{RST}  {DIM}{extras}{RST}")

        print(f"\n{BOLD}{'═'*60}{RST}")
        print(f"  {DIM}Press Ctrl+C to quit{RST}\n")


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
