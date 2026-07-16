import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')

        model_path = os.path.expanduser(
            '~/ros2_ws/src/perception_pkg/perception_pkg/weights/best.pt'
        )
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.conf_threshold = 0.60

        # Görüntü subscriber
        self.subscription = self.create_subscription(
            Image,
            '/arac/kamera/image_raw',
            self.image_callback,
            10
        )

        # Tespit sonucu publisher
        self.pub_sign = self.create_publisher(String, '/detected_sign', 10)

        # Annotated görüntü publisher (RViz için)
        self.pub_image = self.create_publisher(Image, '/sign_detector/annotated', 10)

        self.get_logger().info('Tabela Algılama Düğümü Başlatıldı!')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        detected_labels = []
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                conf = float(box.conf[0])
                detected_labels.append(f'{label}:{conf:.2f}')

        # /detected_sign topic'ine yayınla
        if detected_labels:
            msg_out = String()
            msg_out.data = ', '.join(detected_labels)
            self.pub_sign.publish(msg_out)
            self.get_logger().info(f'Tespit: {msg_out.data}')

        # Annotated görüntüyü yayınla
        annotated = results[0].plot()
        # cv2.imshow kaldirıldı
        # cv2.waitKey kaldirıldı

        ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        ann_msg.header = msg.header
        self.pub_image.publish(ann_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
