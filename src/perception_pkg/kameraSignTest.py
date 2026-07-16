#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class KameraTest(Node):
    def __init__(self):
        super().__init__('kamera_test')
        model_path = os.path.expanduser(
            '~/ros2_ws/src/perception_pkg/perception_pkg/weights/best.pt'
        )
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/kamera_test/annotated', 10)
        self.pub_sign  = self.create_publisher(String, '/kamera_test/detected', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.033, self.callback)
        self.get_logger().info('Kamera testi baslatildi!')

    def callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame, conf=0.75, verbose=False)
        annotated = results[0].plot()

        for box in results[0].boxes:
            label = self.model.names[int(box.cls[0])]
            conf  = float(box.conf[0])
            self.get_logger().info(f'Tespit: {label} - {conf:.2f}')
            msg = String()
            msg.data = f'{label}:{conf:.2f}'
            self.pub_sign.publish(msg)

        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.pub_image.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = KameraTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
