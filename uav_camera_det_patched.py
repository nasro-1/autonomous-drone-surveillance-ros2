#!/usr/bin/env python3
"""
uav_camera_det_patched.py
=========================
DROP-IN replacement for the original uav_camera_det.py.

Changes vs original:
  1. Publishes /target_detected  (std_msgs/Bool)   — True when car/vehicle found
  2. Publishes /target_bbox      (std_msgs/String)  — "cx,cy,w,h" in pixels
  3. Publishes /target_confidence (std_msgs/Float32) — YOLO confidence score
  Everything else (display window, bounding boxes, labels) is unchanged.

Drop this file into ~/PX4-ROS2-Gazebo-YOLOv8/ and run it instead of
uav_camera_det.py.  mission_node.py will subscribe to its output topics.

YOLO classes that count as "target" (vehicles):
  car, truck, bus, motorcycle, bicycle, boat, train, airplane
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO

# ── configurable ──────────────────────────────────────────────────────────────
MODEL_PATH    = "yolov8n.pt"          # or yolov8s.pt for better accuracy
CONF_THRESH   = 0.40
TARGET_CLASSES = {
    "car", "truck", "bus", "motorcycle",
    "bicycle", "boat", "train", "airplane",
    # Gazebo hatchback shows as "car"
}


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        self.bridge = CvBridge()
        self.model  = YOLO(MODEL_PATH)
        self.model.conf = CONF_THRESH

        # subscriber
        self.create_subscription(Image, "/camera", self._cb_image, 10)

        # publishers (NEW — for mission_node)
        self.pub_detected    = self.create_publisher(Bool,    "/target_detected",    10)
        self.pub_bbox        = self.create_publisher(String,  "/target_bbox",        10)
        self.pub_confidence  = self.create_publisher(Float32, "/target_confidence",  10)

        self.get_logger().info("YOLO detector node ready.")

    def _cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(frame, verbose=False)

        best_conf  = 0.0
        best_bbox  = None
        target_found = False

        for result in results:
            for box in result.boxes:
                cls_name = self.model.names[int(box.cls[0])]
                conf     = float(box.conf[0])

                if cls_name not in TARGET_CLASSES:
                    continue

                target_found = True

                # xyxy → cx, cy, w, h
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w  = x2 - x1
                h  = y2 - y1

                if conf > best_conf:
                    best_conf = conf
                    best_bbox = (cx, cy, w, h)

                # Draw on frame
                color = (0, 255, 0) if conf >= CONF_THRESH else (0, 165, 255)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(
                    frame, label, (int(x1), int(y1) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2
                )

                # Crosshair on best target
                if best_bbox and (cx, cy, w, h) == best_bbox:
                    cx_i, cy_i = int(cx), int(cy)
                    cv2.drawMarker(
                        frame, (cx_i, cy_i), (0, 0, 255),
                        cv2.MARKER_CROSS, 30, 2
                    )

        # Image centre cross (helps with alignment)
        h_img, w_img = frame.shape[:2]
        cv2.drawMarker(
            frame, (w_img // 2, h_img // 2), (255, 255, 255),
            cv2.MARKER_CROSS, 20, 1
        )

        # Publish detection results
        det_msg = Bool()
        det_msg.data = target_found
        self.pub_detected.publish(det_msg)

        if best_bbox is not None:
            bbox_msg = String()
            bbox_msg.data = ",".join(f"{v:.2f}" for v in best_bbox)
            self.pub_bbox.publish(bbox_msg)

            conf_msg = Float32()
            conf_msg.data = best_conf
            self.pub_confidence.publish(conf_msg)

        # HUD overlay
        status_text = "TARGET DETECTED" if target_found else "SEARCHING..."
        status_color = (0, 255, 0) if target_found else (0, 200, 255)
        cv2.putText(
            frame, status_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2
        )

        cv2.imshow("UAV Camera — YOLOv8", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
