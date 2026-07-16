#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class SeritTakip(Node):
    def __init__(self):
        super().__init__('serit_takip')
        self.bridge = CvBridge()
        self.mevcut_komut = 'ileri'
        self.komut_sayac  = 0
        self.Kp = 0.008
        self.Ki = 0.0
        self.Kd = 0.003
        self.onceki_hata = 0.0
        self.toplam_hata = 0.0
        self.hiz = 0.8
        self.arac_x = 0.0
        self.viraj_yapildi = False

        self.create_subscription(Image, '/arac/kamera/image_raw', self.goruntu_callback, 10)
        self.create_subscription(String, '/detected_sign', self.tabela_callback, 10)
        self.create_subscription(Odometry, '/arac/odom', self.odom_callback, 10)
        self.cmd_pub   = self.create_publisher(Twist, '/arac/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/serit_debug', 10)
        self.get_logger().info('Serit Takip Dugumu Basladi!')

    def odom_callback(self, msg):
        self.arac_x = msg.pose.pose.position.x
        # Araç x=18'e gelince ve viraj henüz yapılmadıysa sola dön
        if self.arac_x >= 18.0 and not self.viraj_yapildi:
            if self.mevcut_komut == 'ileri':
                self.mevcut_komut = 'sol_don'
                self.komut_sayac  = 50
                self.viraj_yapildi = True
                self.get_logger().info(f'SOLA DON! Arac x={self.arac_x:.1f}')

    def tabela_callback(self, msg):
        data = msg.data
        if 'Stop_Sign' in data:
            self.mevcut_komut = 'dur'
            self.komut_sayac  = 90
            self.get_logger().info('DUR TABELASI! Arac duruyor.')
        elif 'Give Way' in data:
            self.hiz = 0.15
            self.get_logger().info('YOL VER! Yavasladi.')
        elif 'Speed Limit 20' in data:
            self.hiz = 0.2
            self.get_logger().info('HIZ 20.')
        elif 'Speed Limit 30' in data:
            self.hiz = 0.8
            self.get_logger().info('HIZ 30.')
        elif 'No Entry' in data:
            self.mevcut_komut = 'dur'
            self.komut_sayac  = 150
            self.get_logger().info('GIRIS YOK!')
        elif 'Pedestrian Crossing' in data:
            self.hiz = 0.15
            self.get_logger().info('YAYA GECIDI! Yavasladi.')
        elif False:  # Traffic_signal devre disi
            self.hiz = 0.2
            self.get_logger().info('TRAFIK ISIGI! Yavasladi.')

    def goruntu_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w  = frame.shape[:2]
        twist = Twist()

        if self.mevcut_komut == 'dur':
            self.komut_sayac -= 1
            if self.komut_sayac <= 0:
                self.mevcut_komut = 'ileri'
                self.hiz = 0.8
            self.cmd_pub.publish(twist)
            return

        if self.mevcut_komut == 'sol_don':
            self.komut_sayac -= 1
            twist.linear.x  = 0.25
            twist.angular.z = 1.0
            if self.komut_sayac <= 0:
                self.mevcut_komut = 'ileri'
            self.cmd_pub.publish(twist)
            return

        if self.mevcut_komut == 'sag_don':
            self.komut_sayac -= 1
            twist.linear.x  = 0.25
            twist.angular.z = -1.0
            if self.komut_sayac <= 0:
                self.mevcut_komut = 'ileri'
            self.cmd_pub.publish(twist)
            return

        # Serit takibi PID
        roi = frame[h//2:, :]
        rh, rw = roi.shape[:2]
        gri   = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gri, (5, 5), 0)
        kenar = cv2.Canny(blur, 50, 150)
        cizgiler = cv2.HoughLinesP(kenar, 1, np.pi/180,
                    threshold=40, minLineLength=30, maxLineGap=20)
        sol_x_list = []
        sag_x_list = []
        debug_frame = roi.copy()
        merkez_x = rw // 2
        if cizgiler is not None:
            for cizgi in cizgiler:
                x1, y1, x2, y2 = cizgi[0]
                if x2 == x1:
                    continue
                egim = (y2 - y1) / (x2 - x1)
                if abs(egim) < 0.3:
                    continue
                cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                orta_x = (x1 + x2) // 2
                if orta_x < merkez_x:
                    sol_x_list.append(orta_x)
                else:
                    sag_x_list.append(orta_x)
        sol_x = int(np.mean(sol_x_list)) if sol_x_list else 0
        sag_x = int(np.mean(sag_x_list)) if sag_x_list else rw
        serit_merkezi = (sol_x + sag_x) // 2
        hata = merkez_x - serit_merkezi
        self.toplam_hata += hata
        turev_hata = hata - self.onceki_hata
        self.onceki_hata = hata
        donus = self.Kp*hata + self.Ki*self.toplam_hata + self.Kd*turev_hata
        twist.linear.x  = self.hiz
        twist.angular.z = float(np.clip(donus, -1.0, 1.0))
        self.cmd_pub.publish(twist)
        cv2.line(debug_frame, (merkez_x, 0), (merkez_x, rh), (255, 0, 0), 1)
        cv2.line(debug_frame, (serit_merkezi, 0), (serit_merkezi, rh), (0, 0, 255), 2)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SeritTakip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
