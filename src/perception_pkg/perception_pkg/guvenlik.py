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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


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
        # Engelden kacinma: engel seritteyse ve sol serit bossa dur
        # yerine yanindan gec. Kapaliysa katman sadece frenler.
        self.declare_parameter('kacinma_aktif', True)
        self.declare_parameter('kacinma_mesafe', 14.0)   # bu kadar kala karar
        self.declare_parameter('kacinma_ofset', 2.8)     # sola kayma (m)
        self.declare_parameter('serit_genislik', 3.5)

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

        self.kacinma_aktif = g('kacinma_aktif').value
        self.d_kacinma = g('kacinma_mesafe').value
        self.ofset_hedef = g('kacinma_ofset').value
        self.serit_w = g('serit_genislik').value

        self.en_yakin = float('inf')      # koridor icindeki en yakin donus
        self.sol_bos = True               # sol serit bos mu
        self.son_direksiyon = 0.0         # son komut edilen direksiyon acisi
        self.scan_zamani = 0.0
        self.scan_bayat_esigi = 0.5        # 500 ms'den eski scan guvenilmez

        # Kacinma durum makinesi: 'yok' -> 'kaciniyor' -> 'donuyor' -> 'yok'
        self.kacinma_durum = 'yok'
        self.ofset = 0.0                  # mevcut yanal ofset (rampali)
        self.kat_edilen = 0.0             # odom'dan entegre mesafe
        self.onceki_poz = None
        self.kacinma_bitis_mesafe = 0.0   # bu mesafede donuse basla

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/arac/scan', self.scan_cb, qos)
        self.create_subscription(Twist, '/arac/cmd_vel_cmd', self.cmd_cb, 10)
        self.create_subscription(Odometry, '/arac/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/arac/cmd_vel', 10)
        self.ofset_pub = self.create_publisher(Float64, '/arac/kacinma_ofset',
                                               10)
        # Ofset rampasi ve durum makinesi sabit hizda donsun diye timer
        self.create_timer(0.05, self.kacinma_dongusu)

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

        # Sol serit bos mu? Araç sag seritte; sol serit +y tarafinda.
        # Bolge: y in [+1.0, +serit+1.5], ileri x in [-2, d_kacinma+6].
        # Bir sey varsa (ornek karsi yondeki trafik) kacinma guvensiz.
        sol_bolge = (gecerli & (y > 1.0) & (y < self.serit_w + 1.5)
                     & (x > -2.0) & (x < self.d_kacinma + 6.0))
        self.sol_bos = not bool(np.any(sol_bolge))

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

        cikis = Twist()
        cikis.angular.z = msg.angular.z

        # Kacinma manevrasi sirasinda frenleme; araç engeli solundan
        # siyirarak geciyor. Sadece hizi guvenli bir degere kirp.
        if self.kacinma_durum in ('kaciniyor', 'donuyor'):
            cikis.linear.x = min(msg.linear.x, 3.0)
            self.pub.publish(cikis)
            return

        d = self.en_yakin
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

    def odom_cb(self, msg):
        """Kat edilen mesafeyi entegre et (kacinma manevra uzunlugu icin)."""
        p = msg.pose.pose.position
        if self.onceki_poz is not None:
            self.kat_edilen += math.hypot(p.x - self.onceki_poz[0],
                                          p.y - self.onceki_poz[1])
        self.onceki_poz = (p.x, p.y)

    def kacinma_dongusu(self):
        """Engelden kacinma durum makinesi ve yanal ofset rampasi.

        yok -> (engel yakin & sol bos) -> kaciniyor -> (engel gecildi)
            -> donuyor -> (ofset sifir) -> yok

        Erken donusu onlemek icin 'gecildi' karari LiDAR'a degil kat
        edilen mesafeye dayaniyor: araç sola kayinca engel yay
        koridorundan cikip en_yakin=inf oluyor, LiDAR'a bakilsaydi araç
        hemen saga donup engele carpardi.
        """
        RAMPA = 0.08          # her 0.05 s'de ofset degisimi (m)
        yayinla = Float64()

        if not (self.aktif and self.kacinma_aktif):
            yayinla.data = 0.0
            self.ofset_pub.publish(yayinla)
            return

        if self.kacinma_durum == 'yok':
            if (self.en_yakin <= self.d_kacinma and self.sol_bos
                    and self.onceki_poz is not None):
                self.kacinma_durum = 'kaciniyor'
                # engeli gecmek icin gereken yol: mevcut mesafe + engel
                # uzakligi + arac/engel boyu payi
                self.kacinma_bitis_mesafe = (self.kat_edilen
                                             + self.en_yakin + 8.0)
                self.get_logger().warn(
                    f'KACINMA basladi | engel {self.en_yakin:.1f} m, '
                    f'sol serit bos')
        elif self.kacinma_durum == 'kaciniyor':
            self.ofset = min(self.ofset_hedef, self.ofset + RAMPA)
            if self.kat_edilen >= self.kacinma_bitis_mesafe:
                self.kacinma_durum = 'donuyor'
                self.get_logger().warn('KACINMA: engel gecildi, seride donuluyor')
        elif self.kacinma_durum == 'donuyor':
            self.ofset = max(0.0, self.ofset - RAMPA)
            if self.ofset <= 0.01:
                self.ofset = 0.0
                self.kacinma_durum = 'yok'

        yayinla.data = self.ofset
        self.ofset_pub.publish(yayinla)

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
