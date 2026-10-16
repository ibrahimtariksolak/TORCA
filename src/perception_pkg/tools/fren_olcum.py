#!/usr/bin/env python3
"""Aracin GERCEK fren mesafesini ve yavaslamasini olcer.

Guvenlik katmaninin yavaslama_mesafesi parametresi tahminle degil bu
olcumle secilmeli. Arac hedef hiza cikarilir, sonra cmd_vel sifirlanir
ve durana kadar aldigi yol kaydedilir.

Kullanim (simulasyon calisirken, algilama KAPALI olmali):
    ros2 launch perception_pkg otonom_surus.launch.py \\
        gui:=false rviz:=false algilama:=false
    python3 tools/fren_olcum.py [hiz1 hiz2 ...]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Olcum:
    def __init__(self, node):
        self.n = node
        self.pub = node.create_publisher(Twist, '/arac/cmd_vel', 10)
        self.poz = None
        self.hiz = 0.0
        node.create_subscription(Odometry, '/arac/odom', self._odom, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.poz = (p.x, p.y)
        # odom.twist guvenilir degil; hizi konum farkindan turetiyoruz
        # (bkz. tools/direksiyon notlari). Burada sadece referans.
        v = m.twist.twist.linear
        self.hiz = math.hypot(v.x, v.y)

    def _bekle(self, saniye):
        t0 = time.time()
        while time.time() - t0 < saniye:
            rclpy.spin_once(self.n, timeout_sec=0.01)

    def _sur(self, v, sure):
        t = Twist()
        t.linear.x = float(v)
        t0 = time.time()
        while time.time() - t0 < sure:
            self.pub.publish(t)
            rclpy.spin_once(self.n, timeout_sec=0.01)

    def olc(self, hedef_hiz):
        """hedef_hiz'a cik, sonra fren yap; alinan yolu dondur."""
        # hizlan
        self._sur(hedef_hiz, 8.0)
        if self.poz is None:
            return None

        # gercek hizi konum farkindan olc
        p0 = self.poz
        t0 = time.time()
        self._sur(hedef_hiz, 1.0)
        p1 = self.poz
        dt = time.time() - t0
        gercek_v = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / dt

        # FREN: sifir komut
        fren_bas = self.poz
        t_bas = time.time()
        dur = Twist()
        hareketsiz = 0
        onceki = self.poz
        while time.time() - t_bas < 15.0:
            self.pub.publish(dur)
            rclpy.spin_once(self.n, timeout_sec=0.01)
            if self.poz and onceki:
                d = math.hypot(self.poz[0] - onceki[0],
                               self.poz[1] - onceki[1])
                hareketsiz = hareketsiz + 1 if d < 0.001 else 0
                if hareketsiz > 50:
                    break
            onceki = self.poz
        fren_sure = time.time() - t_bas
        mesafe = math.hypot(self.poz[0] - fren_bas[0],
                            self.poz[1] - fren_bas[1])
        ivme = gercek_v / fren_sure if fren_sure > 0 else 0.0
        return gercek_v, mesafe, fren_sure, ivme


def main():
    hizlar = [float(x) for x in sys.argv[1:]] or [3.0, 5.0, 7.0]
    rclpy.init()
    n = rclpy.create_node('fren_olcum')
    o = Olcum(n)
    for _ in range(100):
        rclpy.spin_once(n, timeout_sec=0.02)

    print(f'{"komut":>6} | {"gercek v":>9} | {"fren yolu":>10} | '
          f'{"sure":>6} | {"ort ivme":>9}')
    print('-' * 56)
    sonuc = []
    for h in hizlar:
        r = o.olc(h)
        if r is None:
            print(f'{h:6.1f} | odom alinamadi')
            continue
        v, m, s, a = r
        sonuc.append((v, m))
        print(f'{h:6.1f} | {v:7.2f}   | {m:8.2f}   | {s:5.2f}  | '
              f'{a:7.2f}')
        o._bekle(2.0)

    if sonuc:
        print()
        print('Guvenlik katmani icin oneri:')
        en_hizli_v, en_hizli_m = max(sonuc, key=lambda t: t[0])
        # tepki payi: LiDAR 15 Hz + kontrol dongusu, ~0.4 s say
        tepki = en_hizli_v * 0.4
        print(f'  en yuksek hizda ({en_hizli_v:.1f} m/s) fren yolu '
              f'{en_hizli_m:.1f} m')
        print(f'  tepki payi (0.4 s)                    {tepki:.1f} m')
        print(f'  -> yavaslama_mesafesi >= '
              f'{en_hizli_m + tepki + 2.0:.1f} m (2 m emniyet dahil)')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
