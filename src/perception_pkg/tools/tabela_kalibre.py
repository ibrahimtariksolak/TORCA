#!/usr/bin/env python3
"""Tabela bbox yuksekligi <-> gercek mesafe iliskisini olcer.

sign_detector, bir tabelaya tepki verip vermeyecegine bbox yuksekligine
bakarak karar veriyor. Tabela mesh'leri olcekli oldugundan (ornegin
speed_limit_sign 10x) gercek boyutlarindan metrik mesafe turetmek
guvenilir degil; bu yuzden esik burada DENEYSEL olarak kalibre edilir.

Arac pistte giderken her tespitin bbox yuksekligi, o andaki gercek
tabela mesafesiyle eslestirilir. Cikti, min_bbox_yukseklik parametresi
icin hangi degerin kac metreye denk dustugunu gosterir.

Kullanim (simulasyon calisirken):
    python3 tools/tabela_kalibre.py [saniye]
"""
import math
import os
import sys
import time

# Dunya ureteci bir ROS modulu degil, worlds/ altinda duran bagimsiz bir
# script. Tabela konumlarini birebir ayni kaynaktan almak icin yolu
# elle ekliyoruz; bu yuzden asagidaki importlar dosyanin basinda degil
# (flake8 E402 bilerek susturuldu).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'worlds'))

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

import generate_otoban as go


def tabela_konumlari():
    """Dunya uretecindeki tabela yerlesimini birebir tekrarlar."""
    duz, R, serit = go.DUZ_UZUNLUK, go.VIRAJ_YARICAP, go.SERIT_GENISLIK
    viraj_boy = math.pi * R
    ofset = -(serit + 1.2)
    yerler = {}
    for ad, s in (('dur_tabelasi', duz - 25.0),
                  ('dur_tabelasi_2', duz + viraj_boy + duz / 2.0),
                  ('hiz_limit_gorsel', 2 * duz + viraj_boy + 5.0)):
        x, y, _ = go.ofset_nokta(s, ofset, duz, R)
        yerler[ad] = (x, y)
    return yerler


class Kalibre(Node):
    def __init__(self):
        super().__init__('tabela_kalibre')
        pkg = get_package_share_directory('perception_pkg')
        self.model = YOLO(os.path.join(pkg, 'weights', 'best.pt'))
        self.br = CvBridge()
        self.poz = None
        self.tabelalar = tabela_konumlari()
        self.olcum = []
        self.son = 0.0
        self.create_subscription(Image, '/arac/kamera/image_raw',
                                 self.goruntu_cb, 1)
        self.create_subscription(Odometry, '/arac/odom', self.odom_cb, 10)

    def odom_cb(self, m):
        p = m.pose.pose.position
        self.poz = (p.x, p.y)

    def goruntu_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.poz is None or t - self.son < 0.2:
            return
        self.son = t

        kare = self.br.imgmsg_to_cv2(msg, 'bgr8')
        for r in self.model(kare, conf=0.40, verbose=False):
            for kutu in r.boxes:
                ad = self.model.names[int(kutu.cls[0])]
                x1, y1, x2, y2 = (float(v) for v in kutu.xyxy[0])
                h = y2 - y1
                # en yakin bilinen tabelaya olan mesafe
                en_yakin = min(
                    (math.hypot(self.poz[0] - tx, self.poz[1] - ty), tad)
                    for tad, (tx, ty) in self.tabelalar.items())
                self.olcum.append((en_yakin[0], h, ad, en_yakin[1],
                                   float(kutu.conf[0])))


def main():
    sure = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
    rclpy.init()
    n = Kalibre()
    print('tabela konumlari:')
    for ad, (x, y) in n.tabelalar.items():
        print(f'   {ad:18s} ({x:7.2f}, {y:7.2f})')
    print()

    bas = time.time()
    while time.time() - bas < sure:
        rclpy.spin_once(n, timeout_sec=0.02)

    if not n.olcum:
        print('hic tespit olmadi')
        return

    print(f'toplam tespit: {len(n.olcum)}')
    print()
    print('mesafe araligina gore bbox yuksekligi:')
    print(f'{"mesafe (m)":>12} | {"n":>4} | {"bbox h (px)":>22} | siniflar')
    kenarlar = [0, 10, 15, 20, 25, 30, 40, 60, 100]
    for lo, hi in zip(kenarlar[:-1], kenarlar[1:]):
        grup = [o for o in n.olcum if lo <= o[0] < hi]
        if not grup:
            continue
        h = np.array([o[1] for o in grup])
        siniflar = {}
        for o in grup:
            siniflar[o[2]] = siniflar.get(o[2], 0) + 1
        ust = ', '.join(f'{k} x{v}' for k, v in
                        sorted(siniflar.items(), key=lambda t: -t[1])[:3])
        print(f'{lo:5d}-{hi:<6d} | {len(grup):4d} | '
              f'ort {h.mean():6.1f}  min {h.min():5.1f}  max {h.max():5.1f} | {ust}')

    print()
    print('esik onerisi (min_bbox_yukseklik -> bu mesafeden yakinda tepki):')
    for esik in (20, 25, 30, 40, 50):
        gecen = [o[0] for o in n.olcum if o[1] >= esik]
        if gecen:
            print(f'   {esik:3d} px -> en uzak {max(gecen):5.1f} m, '
                  f'ortalama {np.mean(gecen):5.1f} m, {len(gecen)} tespit')
        else:
            print(f'   {esik:3d} px -> hic tespit gecmiyor')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
