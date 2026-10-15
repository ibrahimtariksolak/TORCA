#!/usr/bin/env python3
"""Serit takibini nicel olarak olcer: arac pistte kaliyor mu, tur atiyor mu?

Kullanim (simulasyon calisirken):
    python3 tools/tur_testi.py [saniye]

Olcut: aracin oval pistin sag serit merkezinden yanal sapmasi. Pist
geometrisi generate_otoban.py ile ayni parametrelerden turetilir, yani
dunyayi degistirirsen buradaki sabitleri de guncelle.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

# generate_otoban.py ile ayni degerler
DUZ = 60.0
R = 25.0
SERIT = 3.5
YARIM = DUZ / 2.0


def orta_hat_mesafe(x, y):
    """Noktanin oval ORTA HATTINA en kisa mesafesi (isaretsiz)."""
    if -YARIM <= x <= YARIM:
        return abs(abs(y) - R)          # duz bolum: alt y=-R, ust y=+R
    cx = YARIM if x > 0 else -YARIM     # viraj merkezi
    return abs(math.hypot(x - cx, y) - R)


def main():
    sure = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    n = rclpy.create_node('tur_testi')
    d = {'ilk': None, 'son': None, 'yol': 0.0, 'onceki': None,
         'sapma_maks': 0.0, 'sapma_top': 0.0, 'n': 0,
         't0': None, 't1': None, 'duz_top': 0.0, 'duz_n': 0,
         'vir_top': 0.0, 'vir_n': 0}

    def cb(m):
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if d['ilk'] is None:
            d['ilk'], d['t0'] = (p.x, p.y), t
        if d['onceki']:
            d['yol'] += math.hypot(p.x - d['onceki'][0], p.y - d['onceki'][1])
        d['onceki'] = d['son'] = (p.x, p.y)
        d['t1'] = t
        s = abs(orta_hat_mesafe(p.x, p.y) - SERIT / 2.0)
        d['sapma_maks'] = max(d['sapma_maks'], s)
        d['sapma_top'] += s
        d['n'] += 1
        if -YARIM <= p.x <= YARIM:
            d['duz_top'] += s
            d['duz_n'] += 1
        else:
            d['vir_top'] += s
            d['vir_n'] += 1

    n.create_subscription(Odometry, '/arac/odom', cb, 10)
    bas = time.time()
    while time.time() - bas < sure:
        rclpy.spin_once(n, timeout_sec=0.1)

    if not d['n']:
        print('odom alinamadi -- simulasyon calisiyor mu?')
        return

    sim = (d['t1'] or 0) - (d['t0'] or 0)
    tur_boy = 2 * DUZ + 2 * math.pi * R
    print(f'sim suresi      : {sim:.1f} s')
    print(f'kat edilen yol  : {d["yol"]:.1f} m   (tam tur = {tur_boy:.1f} m)')
    print(f'tur             : {d["yol"]/tur_boy:.2f}')
    if sim > 0:
        print(f'ort hiz         : {d["yol"]/sim:.2f} m/s')
    print(f'serit sapmasi   : ort {d["sapma_top"]/d["n"]:.2f} m, '
          f'maks {d["sapma_maks"]:.2f} m')
    if d['duz_n']:
        print(f'   duz bolumde  : ort {d["duz_top"]/d["duz_n"]:.2f} m')
    if d['vir_n']:
        print(f'   virajda      : ort {d["vir_top"]/d["vir_n"]:.2f} m')
    print(f'son konum       : {d["son"][0]:.1f}, {d["son"][1]:.1f}')
    print()
    if d['sapma_maks'] < SERIT / 2.0:
        print('>>> SERITTE KALDI (sapma < yarim serit)')
    elif d['sapma_maks'] < SERIT:
        print('>>> serit disina tasti ama yolda')
    else:
        print('>>> PISTTEN CIKTI')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
