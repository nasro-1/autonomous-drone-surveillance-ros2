import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDebug(Node):
    def __init__(self):
        super().__init__('yolo_debug')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.create_subscription(Image, '/camera', self.cb, 10)
        self.get_logger().info('YOLO DEBUG running: conf=0.05, ALL classes')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        r = self.model(frame, conf=0.05, verbose=False)[0]
        if len(r.boxes) == 0:
            self.get_logger().info(f'frame {w}x{h}: NOTHING')
        else:
            for b in r.boxes:
                name = self.model.names[int(b.cls[0])]
                conf = float(b.conf[0])
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                self.get_logger().info(f'{name}  conf={conf:.2f}  size={int(x2-x1)}x{int(y2-y1)}px')
        cv2.imshow('yolo_debug', r.plot())
        cv2.waitKey(1)

def main():
    rclpy.init()
    n = YoloDebug()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
