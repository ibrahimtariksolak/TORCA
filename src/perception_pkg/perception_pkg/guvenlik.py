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
        # Varsayilanlar tools/fren_olcum.py ile OLCULDU:
        #   6.9 m/s -> fren yolu 9.64 m, ortalama ivme 2.00 m/s^2
        # Onceki varsayilanlar (ivme 4.0, yavaslama 6.0 m) fiziksel
        # olarak imkansizdi: araç 7 m/s'de 9.6 m'ye ihtiyac duyarken
        # engeli 6 m'de fark ediyor ve carpiyordu.
        self.declare_parameter('fren_mesafesi_min', 3.0)
        self.declare_parameter('yavaslama_mesafesi', 18.0)
        self.declare_parameter('fren_ivme', 2.0)         # m/s^2, olculdu
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

        # Donus yayini hesaplamak icin dingil mesafesi lazim; URDF ile
        # ayni olmali (bkz. config/params.yaml: serit_takip.dingil_mesafe)
        self.declare_parameter('dingil_mesafe', 2.5)
        self.dingil = self.get_parameter('dingil_mesafe').value

        self.en_yakin = float('inf')      # koridor icindeki en yakin donus
        self.son_direksiyon = 0.0         # son komut edilen direksiyon acisi
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
        """Aracin PLANLANAN YAYI boyunca en yakin engeli bul.

        Koridoru duz ileri almak virajda ise yaramiyor: araç 26.75 m
        yaricapli bir yayda donerken duz ileri bakis dis bariyere
        sqrt(29.75^2 - 26.75^2) = 13.0 m'de carpiyor. yavaslama_mesafesi
        18 m'ye cikarildiginda katman virajda surekli yanlis pozitif
        uretip araci 1.26 m/s'ye dusurmustu (duzde 3.96 m/s).

        Dogrusu: mevcut direksiyon acisindan donus yaricapini cikar
        (R = L / tan(delta)) ve engelleri bu YAYA gore degerlendir.
        Yaydan sapmasi yari_w'den kucuk olanlar tehlikeli; mesafe de
        duz x degil YAY BOYU olarak olculur.
        """
        acilar = (msg.angle_min
                  + np.arange(len(msg.ranges)) * msg.angle_increment)
        menz = np.asarray(msg.ranges)
        gecerli = (np.isfinite(menz) & (menz > msg.range_min)
                   & (menz < msg.range_max))
        self.scan_zamani = self._simdi()
        if not np.any(gecerli):
            self.en_yakin = float('inf')
            return

        x = menz * np.cos(acilar)
        y = menz * np.sin(acilar)

        delta = self.son_direksiyon
        if abs(delta) < 0.02:
            # Duz gidis: klasik dikdortgen koridor
            koridor = gecerli & (np.abs(y) < self.yari_w) & (x > 0.0)
            self.en_yakin = (float(x[koridor].min()) if np.any(koridor)
                             else float('inf'))
            return

        # Donus yayi: merkez arac cercevesinde (0, R)
        R = self.dingil / math.tan(delta)
        r_nokta = np.hypot(x, y - R)
        yaydan_sapma = np.abs(r_nokta - abs(R))

        # Merkeze gore acisal konum; arac (orijin) referans alinir
        phi_arac = math.atan2(-R, 0.0)
        phi_nokta = np.arctan2(y - R, x)
        suprulen = phi_nokta - phi_arac
        if R < 0:                      # saga donus: ters yon
            suprulen = -suprulen
        suprulen = np.mod(suprulen, 2 * math.pi)   # [0, 2pi) -> hep ileri

        yay_boyu = abs(R) * suprulen
        koridor = (gecerli & (yaydan_sapma < self.yari_w)
                   & (yay_boyu < msg.range_max))
        self.en_yakin = (float(yay_boyu[koridor].min()) if np.any(koridor)
                         else float('inf'))

    def cmd_cb(self, msg):
        """Giren komutu guvenlik denetiminden gecirip yayinla."""
        # Bir sonraki taramanin yay hesabi icin direksiyon acisini sakla.
        # (ackermann_drive eklentisi angular.z'yi direksiyon acisi olarak
        # yorumluyor; olcumle dogrulandi.)
        self.son_direksiyon = msg.angular.z

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
