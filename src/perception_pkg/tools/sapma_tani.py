#!/usr/bin/env python3
"""Serit sapmasinin kaynagini ayirir: ALGI mi, KONTROL mu?

Iki bagimsiz olcum karsilastirilir:

  gercek  : odom'dan, pist geometrisi kullanilarak hesaplanan yanal hata
            (pozitif = araç serit merkezinin SOLUNDA)
  algi    : kameradan cikan serit merkezinin arac cercevesindeki yeri
            (merkez Y_p ise araç -Y_p kadar solda demektir)

Ikisi ortusuyorsa algi dogru, sapma kontrolcuden gelir (saf takibin
virajda biraktigi kalici offset). Aralarinda sabit bir fark varsa algi
yanli.
"""
import math
import sys

sys.path.insert(0, '/home/tarik/ros2_ws/install/perception_pkg/'
                   'lib/python3.10/site-packages')

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from perception_pkg.ipm import IPM

DUZ, R, SERIT = 60.0, 25.0, 3.5
YARIM = DUZ / 2.0
SERIT_MERKEZ_R = R + SERIT / 2.0          # 26.75


def gercek_yanal_hata(x, y):
    """Pozitif = araç serit merkezinin solunda (oval merkezine dogru)."""
    if -YARIM <= x <= YARIM:
        r = abs(y)
    else:
        cx = YARIM if x > 0 else -YARIM
        r = math.hypot(x - cx, y)
    return SERIT_MERKEZ_R - r


class Tani(Node):
    def __init__(self):
        super().__init__('sapma_tani')
        self.br = CvBridge()
        self.ipm = None
        self.poz = None
        self.ornek = []
        self.create_subscription(Image, '/arac/kamera/image_raw',
                                 self.goruntu_cb, 1)
        self.create_subscription(Odometry, '/arac/odom', self.odom_cb, 10)
        self.bek_sari, self.bek_beyaz = SERIT / 2.0, -SERIT / 2.0

    def odom_cb(self, m):
        p = m.pose.pose.position
        self.poz = (p.x, p.y)

    def _cizgi(self, kb, beklenen, pencere=1.3):
        kb_y, kb_g = kb.shape[:2]
        nokta, tahmin = [], beklenen
        adim = max(1, int(0.25 / self.ipm.cozunurluk))
        for py in range(kb_y - 1, -1, -adim):
            sol = int(self.ipm.y_to_kb_sutun(tahmin + pencere))
            sag = int(self.ipm.y_to_kb_sutun(tahmin - pencere))
            sol, sag = max(0, min(sol, sag)), min(kb_g, max(sol, sag))
            if sag - sol < 2:
                continue
            idx = np.nonzero(kb[py, sol:sag])[0]
            if idx.size < 2:
                continue
            Y = self.ipm.kb_sutun_to_y(sol + float(idx.mean()))
            nokta.append((self.ipm.kb_satir_to_x(py), Y))
            tahmin = Y
        return nokta

    def goruntu_cb(self, msg):
        if self.poz is None:
            return
        bgr = self.br.imgmsg_to_cv2(msg, 'bgr8')
        if self.ipm is None:
            h, w = bgr.shape[:2]
            self.ipm = IPM(w, h, 1.40, 1.30, 1.95, 0.10,
                           x_min=5.0, x_max=25.0, y_yari=6.0)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        beyaz = cv2.inRange(hsv, (0, 0, 170), (179, 45, 255))
        sari = cv2.inRange(hsv, (18, 90, 90), (38, 255, 255))
        k = np.ones((3, 3), np.uint8)
        beyaz = cv2.morphologyEx(beyaz, cv2.MORPH_OPEN, k)
        sari = cv2.morphologyEx(sari, cv2.MORPH_OPEN, k)

        s_n = self._cizgi(self.ipm.cevir(sari), self.bek_sari)
        if s_n:
            self.bek_sari = float(np.clip(
                0.7 * self.bek_sari + 0.3 * s_n[0][1], -1.0, 4.5))
            self.bek_beyaz = self.bek_sari - SERIT
        b_n = self._cizgi(self.ipm.cevir(beyaz), self.bek_beyaz)
        if b_n and not s_n:
            self.bek_beyaz = float(np.clip(
                0.7 * self.bek_beyaz + 0.3 * b_n[0][1], -4.5, 1.0))

        b = dict((round(X, 2), Y) for X, Y in b_n)
        s = dict((round(X, 2), Y) for X, Y in s_n)
        ortak = sorted(set(b) | set(s))
        if not ortak:
            return
        X0 = ortak[0]                       # en yakin satir
        yari = SERIT / 2.0
        if X0 in b and X0 in s:
            ayrim = s[X0] - b[X0]
            Yp = ((b[X0] + s[X0]) / 2.0 if 0.6 * SERIT < ayrim < 1.5 * SERIT
                  else s[X0] - yari)
            kaynak = 'ikisi' if 0.6 * SERIT < ayrim < 1.5 * SERIT else 'sari*'
        elif X0 in s:
            Yp, kaynak = s[X0] - yari, 'sari'
        else:
            Yp, kaynak = b[X0] + yari, 'beyaz'

        bolum = 'duz' if -YARIM <= self.poz[0] <= YARIM else 'viraj'
        self.ornek.append((gercek_yanal_hata(*self.poz), -Yp, kaynak, X0,
                           bolum))


def main():
    import time
    rclpy.init()
    n = Tani()
    sure = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    bas = time.time()
    while time.time() - bas < sure:
        rclpy.spin_once(n, timeout_sec=0.02)

    if len(n.ornek) < 20:
        print(f'yetersiz ornek ({len(n.ornek)})')
        return

    def rapor(baslik, ornekler):
        if not ornekler:
            print(f'{baslik}: ornek yok')
            return
        g = np.array([o[0] for o in ornekler])
        a = np.array([o[1] for o in ornekler])
        f = a - g
        print(f'--- {baslik}  (n={len(ornekler)})')
        print(f'    GERCEK yanal hata : ort {g.mean():+.2f}  |ort| {np.abs(g).mean():.2f}  std {g.std():.2f}')
        print(f'    ALGININ gordugu   : ort {a.mean():+.2f}  std {a.std():.2f}')
        print(f'    FARK (algi-gercek): ort {f.mean():+.2f}  std {f.std():.2f}')
        kay = {}
        for o in ornekler:
            kay[o[2]] = kay.get(o[2], 0) + 1
        print('    kaynak: ' + ', '.join(
            f'{k} %{100*v/len(ornekler):.0f}'
            for k, v in sorted(kay.items(), key=lambda t: -t[1])))

    rapor('TUMU', n.ornek)
    print()
    rapor('DUZ BOLUM', [o for o in n.ornek if o[4] == 'duz'])
    print()
    rapor('VIRAJ', [o for o in n.ornek if o[4] == 'viraj'])

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
