#!/usr/bin/env python3
"""Guvenlik katmani: LiDAR tabanli acil fren.

Serit takip kontrolcusunden BAGIMSIZ bir node. /arac/scan'i izler ve
araç yolunda bir engel varsa /arac/cmd_vel uzerindeki hiz komutunu
siniryabilir ya da sifirlar. Kontrolcu normal akisla calisiyor, bu
dugum sadece son savunma hatti.

Mimari: cmd_vel'de iki yayinci olmasini onlemek icin kontrolcunun
cikisi /arac/cmd_vel_cmd topic'ine yonlendirilir (eski /arac/cmd_vel
yerine). Bu node /arac/cmd_vel_cmd'yi dinleyip filtreleyerek
/arac/cmd_vel'e yayinlar.

HAYIR, bu katman devre disi birakilabilir (guvenlik_aktif parametresi).
Boyle bir anahtarin kapatilabilir olmasi tartismali ama cesitli test
senaryolarinda (ornegin kalibrasyon icin sabit tutmak) gerekiyor.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Guvenlik(Node):

    def __init__(self):
        super().__init__('guvenlik')

        # Araç govdesi genisligi iz_genisligi + tekerlek_kalinligi'na gore
        # 1.6 + 0.25 = 1.85 m. Yarim: 0.925 m. Fiziksel genislikten biraz
        # genis bir koridor kontrol ediyoruz.
        self.declare_parameter('koridor_yari_genislik', 1.20)
        # Acil fren mesafesi dinamik hesaplaniyor: en kotu durumda
        # fren ivmesi 4 m/s^2. v=4 m/s icin durma mesafesi 2 m,
        # tepki payiyla ~3 m. Baslangic degeri olarak:
        self.declare_parameter('fren_mesafesi_min', 2.5)
        self.declare_parameter('yavaslama_mesafesi', 6.0)
        self.declare_parameter('fren_ivme', 4.0)         # m/s^2
        self.declare_parameter('aktif', True)
        self.declare_parameter('log_hz', 2.0)

        g = self.get_parameter
        self.yari_w = g('koridor_yari_genislik').value
        self.d_fren_min = g('fren_mesafesi_min').value
        self.d_yavas = g('yavaslama_mesafesi').value
        self.ivme = g('fren_ivme').value
        self.aktif = g('aktif').value
        self.log_min_aralik = 1.0 / max(1e-3, g('log_hz').value)
        self.son_log = 0.0

        self.en_yakin = float('inf')      # koridor icindeki en yakin donus
        self.scan_zamani = 0.0
        self.scan_bayat_esigi = 0.5        # 500 ms'den eski scan guvenilmez

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/arac/scan', self.scan_cb, qos)
        self.create_subscription(Twist, '/arac/cmd_vel_cmd', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/arac/cmd_vel', 10)

        self.get_logger().info(
            f'Guvenlik katmani basladi | aktif={self.aktif} | '
            f'koridor +-{self.yari_w:.2f} m | yavasla<{self.d_yavas:.1f} m | '
            f'dur<{self.d_fren_min:.1f} m')

    def _simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def scan_cb(self, msg):
        """Koridor icindeki (|y| < yari_w) en yakin donusu bul.

        LiDAR arac cercevesinde onde ve ortada. Her isinin uc noktasi
        (r cos a, r sin a); koridor filtresi |r sin a| < yari_w.
        """
        acilar = (msg.angle_min
                  + np.arange(len(msg.ranges)) * msg.angle_increment)
        menz = np.asarray(msg.ranges)
        gecerli = np.isfinite(menz) & (menz > msg.range_min) & (menz < msg.range_max)
        if not np.any(gecerli):
            self.en_yakin = float('inf')
            self.scan_zamani = self._simdi()
            return

        y = menz * np.sin(acilar)
        x = menz * np.cos(acilar)
        koridor = gecerli & (np.abs(y) < self.yari_w) & (x > 0.0)
        if not np.any(koridor):
            self.en_yakin = float('inf')
        else:
            self.en_yakin = float(x[koridor].min())
        self.scan_zamani = self._simdi()

    def cmd_cb(self, msg):
        """Giren komutu guvenlik denetiminden gecirip yayinla."""
        if not self.aktif:
            self.pub.publish(msg)
            return

        # Scan bayatsa ihtiyatli davran: hiz komutunu kirp
        scan_yasi = self._simdi() - self.scan_zamani
        if scan_yasi > self.scan_bayat_esigi:
            cikis = Twist()
            cikis.linear.x = min(msg.linear.x, 1.0)
            cikis.angular.z = msg.angular.z
            self.pub.publish(cikis)
            self._not('scan bayat', scan_yasi)
            return

        d = self.en_yakin
        cikis = Twist()
        cikis.angular.z = msg.angular.z

        if d <= self.d_fren_min:
            cikis.linear.x = 0.0
            self._not('ACIL FREN', d)
        elif d <= self.d_yavas:
            # d_fren_min -> 0 m/s,  d_yavas -> istenen hiz
            oran = (d - self.d_fren_min) / (self.d_yavas - self.d_fren_min)
            # ayrica fren ivmesine uygun ust sinir:
            # v^2 <= 2*a*(d - d_fren_min)
            v_max = math.sqrt(max(0.0, 2.0 * self.ivme * (d - self.d_fren_min)))
            cikis.linear.x = min(msg.linear.x, oran * msg.linear.x, v_max)
            self._not(f'yavasla v={cikis.linear.x:.2f}', d)
        else:
            cikis.linear.x = msg.linear.x

        self.pub.publish(cikis)

    def _not(self, etiket, deger):
        simdi = self._simdi()
        if simdi - self.son_log >= self.log_min_aralik:
            self.get_logger().warn(f'{etiket} (mesafe/yas = {deger:.2f})')
            self.son_log = simdi


def main(args=None):
    rclpy.init(args=args)
    d = Guvenlik()
    try:
        rclpy.spin(d)
    except KeyboardInterrupt:
        pass
    finally:
        d.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
