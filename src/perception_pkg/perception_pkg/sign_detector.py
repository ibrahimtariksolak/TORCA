#!/usr/bin/env python3
"""Trafik tabelasi algilama (YOLOv8).

Onceki surumun sorunlari
------------------------
1. Model yolu '~/ros2_ws/src/...' olarak sabitti. Paket baska bir
   makinede ya da install uzayindan calistirildiginda model bulunamiyordu.
2. Her kamera karesinde (30 Hz) SENKRON cikarim yapiliyordu. Callback
   bloke oluyor, kuyruk birikiyor ve kontrolcu bayat karelere gore
   karar veriyordu. Uc ornek birden acildiginda Gazebo'nun gercek
   zaman carpani yere seriliyordu.
3. Mesafe/boyut suzgeci yoktu: ufuktaki bir tabela, 50 m otedeyken
   araci frenletiyordu.
4. Sinif suzgeci yoktu. Model 29 sinif biliyor ama dunyada bunlarin
   ancak birkaci var; olcumde 'Traffic_signal' surekli yanlis pozitif
   uretiyordu (0.61-0.71 guven).
5. Her tespitte INFO log basiliyordu.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class SignDetector(Node):

    def __init__(self):
        super().__init__('sign_detector')

        pkg_share = get_package_share_directory('perception_pkg')
        varsayilan_model = os.path.join(pkg_share, 'weights', 'best.pt')

        self.declare_parameter('model_yolu', varsayilan_model)
        self.declare_parameter('guven_esigi', 0.60)
        self.declare_parameter('cikarim_hz', 6.0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '')          # '' = otomatik
        # Kalibrasyonla belirlendi (tools/tabela_kalibre.py): gercek
        # Stop_Sign tespitlerinin bbox yuksekligi 47.6-106.9 px, uzaktaki
        # yanlis pozitifler ise <=33.7 px. 35 px ikisini temiz ayiriyor.
        self.declare_parameter('min_bbox_yukseklik', 35)
        self.declare_parameter('ilgili_siniflar', [
            'Stop_Sign', 'No Entry', 'Give Way', 'Pedestrian Crossing',
            'Speed Limit 20 KMPh', 'Speed Limit 30 KMPh',
            'Dangerous Left Curve Ahead', 'Dangerous Rright Curve Ahead',
            'Round-About',
        ])
        self.declare_parameter('annotated_yayinla', True)

        g = self.get_parameter
        model_yolu = g('model_yolu').value
        if not os.path.isfile(model_yolu):
            # kaynak agacindan calistirilma durumu
            yedek = os.path.join(os.path.dirname(__file__), 'weights', 'best.pt')
            if os.path.isfile(yedek):
                model_yolu = yedek
            else:
                raise FileNotFoundError(
                    f'YOLO agirligi bulunamadi: {model_yolu}')

        self.guven = g('guven_esigi').value
        self.imgsz = g('imgsz').value
        self.device = g('device').value or None
        self.min_h = g('min_bbox_yukseklik').value
        self.ilgili = set(g('ilgili_siniflar').value)
        self.annotated_yayinla = g('annotated_yayinla').value

        self.model = YOLO(model_yolu)
        self.bridge = CvBridge()

        periyot = 1.0 / max(1e-3, g('cikarim_hz').value)
        self.min_aralik = periyot
        self.son_cikarim = 0.0
        self.son_yayin = ''

        # Sensor QoS: derinlik 1 + best effort. Kuyruk birikmesin,
        # cikarim yetisemedigi karelerde en guncel kare islensin.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/arac/kamera/image_raw',
                                 self.goruntu_cb, qos)
        self.pub_sign = self.create_publisher(String, '/detected_sign', 10)
        self.pub_image = self.create_publisher(
            Image, '/sign_detector/annotated', 10)

        self.get_logger().info(
            f'Tabela algilama basladi | model={os.path.basename(model_yolu)} '
            f'| {g("cikarim_hz").value:.0f} Hz | guven>={self.guven} '
            f'| min bbox yuksekligi={self.min_h}px')

    def _simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def goruntu_cb(self, msg):
        simdi = self._simdi()
        if simdi - self.son_cikarim < self.min_aralik:
            return                      # throttle: her kareyi isleme
        self.son_cikarim = simdi

        kare = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        sonuc = self.model(kare, conf=self.guven, imgsz=self.imgsz,
                           device=self.device, verbose=False)

        etiketler = []
        for r in sonuc:
            for kutu in r.boxes:
                ad = self.model.names[int(kutu.cls[0])]
                if self.ilgili and ad not in self.ilgili:
                    continue            # dunyada olmayan sinifi yok say
                x1, y1, x2, y2 = (float(v) for v in kutu.xyxy[0])
                yukseklik = y2 - y1
                # Yakinlik olcutu: bbox yuksekligi. Tabela mesh'leri
                # olcekli oldugu icin gercek boyutlarindan metrik mesafe
                # turetmek guvenilir degil; bunun yerine esik deneysel
                # kalibre edildi (bkz. tools/tabela_kalibre.py).
                if yukseklik < self.min_h:
                    continue            # cok uzak, henuz tepki verme
                etiketler.append(f'{ad}:{float(kutu.conf[0]):.2f}')

        if etiketler:
            cikti = ', '.join(etiketler)
            m = String()
            m.data = cikti
            self.pub_sign.publish(m)
            # Log spam'i onle: sadece tespit degistiginde yaz
            if cikti != self.son_yayin:
                self.get_logger().info(f'Tespit: {cikti}')
                self.son_yayin = cikti
        else:
            self.son_yayin = ''

        if self.annotated_yayinla:
            ann = sonuc[0].plot()
            am = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')
            am.header = msg.header
            self.pub_image.publish(am)


def main(args=None):
    rclpy.init(args=args)
    dugum = SignDetector()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
