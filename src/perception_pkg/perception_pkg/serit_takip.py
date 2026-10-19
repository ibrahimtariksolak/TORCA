#!/usr/bin/env python3
"""Serit takibi: HSV maskesi + kus bakisi + saf takip (pure pursuit).

Onceki surumun sorunlari
------------------------
1. Canny + HoughLinesP ham kamera goruntusunde calisiyordu. Olcumde
   TEK bir cizgi bile bulunamadi: iki liste de bos kalinca sol_x=0,
   sag_x=genislik oluyor, serit merkezi tam goruntu merkezine dusuyor
   ve hata SIFIR cikiyordu. Araç serit takip ettigini sanarak tam gazda
   duz gidiyor, ilk virajda pistten cikiyordu.
2. Sol/sag ayrimi goruntu merkezine gore yapiliyordu. Virajda her iki
   cizgi de ayni tarafa dustugunde siniflandirma tamamen bozuluyor.
3. Perspektif duzeltmesi yoktu; piksel hatasi gercek yanal hataya
   dogrusal degildi, efektif kazanc mesafeyle degisiyordu.
4. PID'de dt yoktu, integral sinirsizdi, serit kaybinda davranis yoktu.

Bu surum
--------
- Renk maskeleri: sari = orta cizgi (solumuzda), beyaz = dis kenar
  (sagimizda). Dunya bu iki rengi ayirarak uretildigi icin siniflandirma
  egim tahminine degil renge dayaniyor; virajda bozulmuyor.
- Kus bakisi (IPM) ile metrik uzayda calisiliyor.
- Kayan pencere: her cizgi bir onceki tahminin etrafinda aranir, boylece
  ic kenar cizgisi (bizden +5.25 m) dis kenarla karistirilmaz.
- Saf takip kontrolcusu DOGRUDAN DIREKSIYON ACISI uretir. Olcumle
  dogrulandi: ackermann_drive eklentisi cmd_vel.angular.z'yi yonelme
  hizi degil direksiyon acisi olarak yorumluyor.
- Serit kaybinda yavasla, sureklilesirse dur.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

from perception_pkg.ipm import IPM


class SeritTakip(Node):

    def __init__(self):
        super().__init__('serit_takip')

        # ---- parametreler ----
        self.declare_parameter('dingil_mesafe', 2.5)
        self.declare_parameter('maks_direksiyon', 0.6)
        self.declare_parameter('serit_genislik', 3.5)
        self.declare_parameter('hiz_duz', 7.0)
        self.declare_parameter('hiz_viraj', 3.5)
        self.declare_parameter('on_bakis_kat', 1.4)   # x hiz
        self.declare_parameter('on_bakis_min', 6.0)
        self.declare_parameter('on_bakis_maks', 18.0)
        self.declare_parameter('kamera_yuksek', 1.30)
        # urdf'teki kamera_ileri ile AYNI olmali; IPM homografisi buna bagli
        self.declare_parameter('kamera_ileri', 1.95)
        self.declare_parameter('kamera_egim', 0.10)
        self.declare_parameter('yatay_fov', 1.40)
        self.declare_parameter('debug_yayinla', True)

        g = self.get_parameter
        self.L = g('dingil_mesafe').value
        self.maks_dir = g('maks_direksiyon').value
        self.serit_w = g('serit_genislik').value
        self.hiz_duz = g('hiz_duz').value
        self.hiz_viraj = g('hiz_viraj').value
        self.ob_kat = g('on_bakis_kat').value
        self.ob_min = g('on_bakis_min').value
        self.ob_maks = g('on_bakis_maks').value
        self.debug_yayinla = g('debug_yayinla').value

        self.ipm = None          # ilk karede goruntu boyutuyla kurulur
        self._ipm_params = (g('yatay_fov').value, g('kamera_yuksek').value,
                            g('kamera_ileri').value, g('kamera_egim').value)

        # ---- durum ----
        self.bridge = CvBridge()
        self.hiz_limiti = self.hiz_duz
        self.mevcut_komut = 'ileri'
        self.komut_biter = 0.0
        self.tabela_kilidi = {}
        self.beklenen_sari = +self.serit_w / 2.0
        self.beklenen_beyaz = -self.serit_w / 2.0
        self.suzulmus_egri = 0.0
        self.kayip_sayac = 0
        self.son_direksiyon = 0.0

        # ---- arayuz ----
        # Goruntu icin sensor QoS: gecikme birikmesin, bayat kare
        # uzerinde karar verilmesin.
        kamera_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/arac/kamera/image_raw',
                                 self.goruntu_cb, kamera_qos)
        self.create_subscription(String, '/detected_sign', self.tabela_cb, 10)
        self.create_subscription(Odometry, '/arac/odom', self.odom_cb, 10)
        # Engelden kacinma yanal ofseti: guvenlik dugumu sol seridin bos
        # oldugunu dogrulayinca buraya pozitif bir deger yolluyor, araç
        # o kadar sola kayiyor (bkz. _saf_takip).
        self.create_subscription(Float64, '/arac/kacinma_ofset',
                                 self.ofset_cb, 10)
        # Cikis /arac/cmd_vel_cmd'ye gidiyor; guvenlik dugumu (varsa)
        # bunu /arac/cmd_vel'e filtreleyerek yayinliyor. guvenlik dugumu
        # kapaliysa launch bir remap koyarak cmd_vel_cmd'yi dogrudan
        # cmd_vel'e yonlendiriyor.
        self.cmd_pub = self.create_publisher(Twist, '/arac/cmd_vel_cmd', 10)
        self.debug_pub = self.create_publisher(Image, '/serit_debug', 10)

        # ackermann_drive eklentisi ILK cmd_vel mesajini alana kadar
        # araci surukluyor (hedef degiskenleri o ana dek tanimsiz).
        # Acilista sifir basarak sabitliyoruz.
        self.cmd_pub.publish(Twist())

        self.arac_hiz = 0.0
        self.arac_yaw = 0.0
        self.yaya_biter = 0.0
        self.yaya_sayac = 0
        self.kacinma_ofset = 0.0
        # Sagdon manevrasi durum degiskenleri
        self.sagdon_faz = 0            # 0 = aktif degil
        self.sagdon_hedef_yaw = 0.0
        self.sagdon_faz_bas = 0.0      # faz basladigi an (fallback icin)
        self.get_logger().info('Serit takip basladi (IPM + saf takip)')

    def ofset_cb(self, msg):
        self.kacinma_ofset = float(msg.data)

    # ================= sabitler =================
    DUR_SURESI = 3.0
    GIRIS_YOK_SURESI = 5.0
    TABELA_BEKLEME = 8.0
    KAYIP_YAVASLA = 5          # kac kare sonra yavaslamaya basla
    KAYIP_DUR = 45             # kac kare sonra tamamen dur
    YAYA_ESIK = 0.13           # serit ici beyaz orani
    YAYA_ONAY = 2              # kac ardisik kare gerekli (transient'i ele)
    YAYA_HIZ = 2.0             # yaya gecidinde hiz limiti (m/s)
    YAYA_SURE = 4.0            # yavaslamanin suresi (s)

    # Sagdon manevrasi: Turn right ahead tabelasina tepki.
    # Toplam 360 derece dagilimla U-donusu: 90 + 180 + 90.
    # Araç orijinal yonune doner, pistin ~15 m ilerisine yakin kalir.
    # Her 'donus' fazi hedef yaw'a ulasana kadar sabit sag direksiyon,
    # her 'duz' fazi sabit sure boyunca direksiyon sifir.
    SAGDON_DIR = -0.5          # sabit direksiyon acisi (rad, negatif = sag)
    SAGDON_HIZ = 2.0           # manevra sirasindaki hiz (m/s)
    SAGDON_DUZ_SURE = 2.0      # fazlar arasi duz gidis suresi (s)
    SAGDON_YAW_TOL = 0.08      # hedef yaw'a yaklasim toleransi (rad, ~5°)
    SAGDON_FAZ_MAKS = 10.0     # tek fazin azami suresi (fallback / takilma)
    SAGDON_DONUS_DIR = 0.25    # yola donus: hafif sol direksiyon (rad)
    SAGDON_DONUS_MAKS = 8.0    # yola donus maks suresi (s)

    # ================= yardimcilar =================
    def _simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tetiklenebilir(self, etiket):
        """Ayni tabela goruste kaldigi surece komutu tazelemesini onler.

        Onceki surumde bu kilit yoktu: araç DUR tabelasini gorup duruyor,
        tabela hala karede oldugu icin her goruntude sayac basa doniyor
        ve araç tabelanin onunde kalici olarak kilitleniyordu.
        """
        s = self._simdi()
        if s < self.tabela_kilidi.get(etiket, 0.0):
            return False
        self.tabela_kilidi[etiket] = s + self.TABELA_BEKLEME
        return True

    def _komut_ver(self, komut, sure):
        self.mevcut_komut = komut
        self.komut_biter = self._simdi() + sure

    @staticmethod
    def _yaw_mod(a):
        """Yaw'i [-pi, pi) araligina indirger."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _sagdon_tick(self):
        """Sagdon manevrasinin bir adimini uretir -> (hiz, direksiyon).

        Fazlar:
            1: yavasla — SAGDON_HIZ'a dusene kadar duz git
            2: 90° sag  -> hedef = baslangic - 90°
            3: 2 s duz
            4: 180° sag -> hedef = fazbasi - 180°
            5: 2 s duz
            6: 90° sag  -> hedef = fazbasi - 90°
            7: yola donus — hafif sol kir, serit bulununca veya sure dolunca bit
            8: bitti, serit takibine don

        Her donus fazi hedef yaw'a tolerans icinde yaklasana kadar
        suruyor, SAGDON_FAZ_MAKS sinirini asarsa fallback olarak faz
        geciyor (araç takilmasin).
        """
        simdi = self._simdi()
        gecen = simdi - self.sagdon_faz_bas

        # Faz 1: yavasla
        if self.sagdon_faz == 1:
            if self.arac_hiz <= self.SAGDON_HIZ + 0.3 or gecen > 4.0:
                self.sagdon_faz = 2
                self.sagdon_faz_bas = simdi
                self.sagdon_hedef_yaw = self._yaw_mod(
                    self.arac_yaw - math.pi / 2.0)
                return self._sagdon_tick()
            return self.SAGDON_HIZ, 0.0

        if self.sagdon_faz in (2, 4, 6):
            # Donus fazi
            yaw_fark = abs(self._yaw_mod(
                self.arac_yaw - self.sagdon_hedef_yaw))
            if yaw_fark < self.SAGDON_YAW_TOL or gecen > self.SAGDON_FAZ_MAKS:
                self.sagdon_faz += 1
                self.sagdon_faz_bas = simdi
                return self._sagdon_tick()
            return self.SAGDON_HIZ, self.SAGDON_DIR

        if self.sagdon_faz in (3, 5):
            # Duz git fazi
            if gecen >= self.SAGDON_DUZ_SURE:
                self.sagdon_faz += 1
                self.sagdon_faz_bas = simdi
                donus = (math.pi if self.sagdon_faz == 4
                         else math.pi / 2.0)
                self.sagdon_hedef_yaw = self._yaw_mod(
                    self.arac_yaw - donus)
                return self._sagdon_tick()
            return self.SAGDON_HIZ, 0.0

        # Faz 7: yola donus — hafif sol kirilarak yola yaklas
        if self.sagdon_faz == 7:
            if gecen > self.SAGDON_DONUS_MAKS:
                self.sagdon_faz = 8
                self.sagdon_faz_bas = simdi
                return self._sagdon_tick()
            return self.SAGDON_HIZ, self.SAGDON_DONUS_DIR

        # Faz 8: manevra tamam
        self.sagdon_faz = 0
        self.get_logger().info('SAGA DON manevra tamam, serit takibine don')
        return None

    def _yaya_gecidi_orani(self, kb_beyaz):
        """Serit ICINDEKI (kenarlar haric) beyaz piksel orani.

        Zebra bantlari serit icini doldurur; normal yolda bu bolge
        asfalt (karanlik). Oran yuksekse yaya gecidi var demektir.

        ROI: yanal olarak kenar cizgilerinin icinde (y in [-1.1, +1.1]),
        ileri olarak 6-14 m penceresi.
        """
        y_sol = int(self.ipm.y_to_kb_sutun(+1.1))
        y_sag = int(self.ipm.y_to_kb_sutun(-1.1))
        y_sol, y_sag = max(0, min(y_sol, y_sag)), min(kb_beyaz.shape[1],
                                                      max(y_sol, y_sag))
        x_yakin = int((self.ipm.x_max - 6.0) / self.ipm.cozunurluk)
        x_uzak = int((self.ipm.x_max - 14.0) / self.ipm.cozunurluk)
        r0, r1 = max(0, x_uzak), min(kb_beyaz.shape[0], x_yakin)
        if r1 - r0 < 2 or y_sag - y_sol < 2:
            return 0.0
        roi = kb_beyaz[r0:r1, y_sol:y_sag]
        return float((roi > 0).mean())

    def odom_cb(self, msg):
        v = msg.twist.twist.linear
        self.arac_hiz = math.hypot(v.x, v.y)
        # yaw: sagdon_manevra fazlari odometriden izleniyor
        q = msg.pose.pose.orientation
        self.arac_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def tabela_cb(self, msg):
        d = msg.data
        if 'Stop_Sign' in d and self._tetiklenebilir('dur'):
            self._komut_ver('dur', self.DUR_SURESI)
            self.get_logger().info('DUR tabelasi')
        elif 'No Entry' in d and self._tetiklenebilir('giris_yok'):
            self._komut_ver('dur', self.GIRIS_YOK_SURESI)
            self.get_logger().info('GIRIS YOK')
        elif 'Give Way' in d and self._tetiklenebilir('yol_ver'):
            self.hiz_limiti = 2.0
            self.get_logger().info('YOL VER')
        elif 'Pedestrian Crossing' in d and self._tetiklenebilir('yaya'):
            self.hiz_limiti = 2.0
            self.get_logger().info('YAYA GECIDI')
        elif 'Speed Limit 20' in d and self._tetiklenebilir('hiz20'):
            self.hiz_limiti = 2.5
            self.get_logger().info('HIZ 20')
        elif (('Turn right ahead' in d or 'Turn left ahead' in d)
              and self._tetiklenebilir('sagdon')
              and self.sagdon_faz == 0):
            # Scripted 360° U-donusu baslat: 90° sag + duz + 180° sag +
            # duz + 90° sag -> arac orijinal yonunde, ileride ana yola
            # yakin kalir. Mevcut yaw baslangic alinir.
            self.sagdon_faz = 1
            self.sagdon_faz_bas = self._simdi()
            self.get_logger().info('SAGA DON manevra basladi, yavaslaniyor')
        elif 'Speed Limit 30' in d and self._tetiklenebilir('hiz30'):
            self.hiz_limiti = self.hiz_duz
            self.get_logger().info('HIZ 30')

    # ================= serit tespiti =================
    @staticmethod
    def _maskeler(bgr):
        """Beyaz kenar cizgisi ve sari orta cizgi maskeleri.

        Canny yerine renk esigi: Canny asfalt dokusunun kendisinde de
        kenar buluyor, renk esigi ise cizgiyi dogrudan ayiriyor.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        beyaz = cv2.inRange(hsv, (0, 0, 170), (179, 45, 255))
        sari = cv2.inRange(hsv, (18, 90, 90), (38, 255, 255))
        cekirdek = np.ones((3, 3), np.uint8)
        beyaz = cv2.morphologyEx(beyaz, cv2.MORPH_OPEN, cekirdek)
        sari = cv2.morphologyEx(sari, cv2.MORPH_OPEN, cekirdek)
        return beyaz, sari

    @staticmethod
    def _kumeler(satir, bosluk=3):
        """Bir maske satirindaki ardisik piksel kumelerinin merkezleri."""
        idx = np.nonzero(satir)[0]
        if idx.size == 0:
            return []
        kopus = np.nonzero(np.diff(idx) > bosluk)[0]
        parcalar = np.split(idx, kopus + 1)
        return [float(p.mean()) for p in parcalar if p.size >= 2]

    def _yol_kenarlari(self, kb_beyaz):
        """Her satirda TUM beyaz kumeleri bulup DIS kenari secer.

        Kesikli sari orta cizgiye bel baglamak calismiyordu: duz
        bolumde (3 m cizgi + 6 m bosluk) karelerin ancak %2'sinde
        goruntuye giriyor, capa olmayinca beyaz takipcisi yanlis
        cizgiye kayiyor ve araci kars'i seride suruyordu (olcum: duz
        bolumde gercek yanal hata 3.22 m, virajda 0.11 m).

        Yolun yapisi ise belirsiz degil: HER ZAMAN iki beyaz kenar var
        ve aralari tam bir yol genisligi (2 x serit). Ayrimi buna uyan
        cifti bulup kucuk Y'li olani (aracin saginda kalan) dis kenar
        kabul ediyoruz. Araç yanlis seritteyse bile cift dogru
        bulunur ve merkez onu geri cagirir.
        """
        yol_w = 2.0 * self.serit_w
        kb_y, kb_g = kb_beyaz.shape[:2]
        adim = max(1, int(0.25 / self.ipm.cozunurluk))

        nokta, tahmin = [], self.beklenen_beyaz
        for py in range(kb_y - 1, -1, -adim):
            merkezler = [self.ipm.kb_sutun_to_y(px)
                         for px in self._kumeler(kb_beyaz[py])]
            if not merkezler:
                continue
            merkezler.sort()

            secim = None
            if len(merkezler) >= 2:
                # ayrimi yol genisligine en yakin cift
                en_iyi, en_iyi_hata = None, 1e9
                for i in range(len(merkezler)):
                    for j in range(i + 1, len(merkezler)):
                        hata = abs((merkezler[j] - merkezler[i]) - yol_w)
                        if hata < en_iyi_hata:
                            en_iyi_hata, en_iyi = hata, merkezler[i]
                if en_iyi_hata < 0.25 * yol_w:
                    secim = en_iyi            # kucuk Y = dis kenar
            if secim is None:
                # tek cizgi: onceki tahmine en yakin olani al
                secim = min(merkezler, key=lambda y: abs(y - tahmin))
                if abs(secim - tahmin) > 2.0:
                    continue

            nokta.append((self.ipm.kb_satir_to_x(py), secim))
            tahmin = secim
        return nokta

    def _cizgi_noktalari(self, kb_maske, beklenen_y, pencere=1.3):
        """Kus bakisi maskeden (X, Y) nokta listesi cikarir.

        Her satir bir ileri mesafedir. Beklenen yanal konumun etrafinda
        +-pencere metrelik bir seritte arama yapilir; boylece ic kenar
        cizgisi (bizden +5.25 m) dis kenarla (-1.75 m) karismaz.
        """
        kb_y, kb_g = kb_maske.shape[:2]
        noktalar = []
        y_tahmin = beklenen_y
        adim = max(1, int(0.25 / self.ipm.cozunurluk))   # ~25 cm'de bir

        # yakindan uzaga: en guvenilir olcumle basla
        for py in range(kb_y - 1, -1, -adim):
            sol = int(self.ipm.y_to_kb_sutun(y_tahmin + pencere))
            sag = int(self.ipm.y_to_kb_sutun(y_tahmin - pencere))
            sol, sag = max(0, min(sol, sag)), min(kb_g, max(sol, sag))
            if sag - sol < 2:
                continue
            satir = kb_maske[py, sol:sag]
            idx = np.nonzero(satir)[0]
            if idx.size < 2:
                continue
            px = sol + float(idx.mean())
            Y = self.ipm.kb_sutun_to_y(px)
            X = self.ipm.kb_satir_to_x(py)
            noktalar.append((X, Y))
            y_tahmin = Y        # pencereyi kaydir
        return noktalar

    def _serit_merkezi(self, kb_beyaz, kb_sari):
        """Serit merkez noktalarini (X, Y) olarak dondurur.

        BIRINCIL REFERANS BEYAZ DIS KENAR, sari degil. Sari orta cizgi
        kesiklidir (3 m cizgi + 6 m bosluk) ve olcumde duz bolumde
        karelerin ancak %2'sinde goruntuye giriyordu; capa olmadigindan
        araç duz bolumde kars'i seride geciyor, algi bunu fark etmiyordu
        (gercek yanal hata: duzde 3.22 m, virajda 0.11 m).

        Beyaz kenarlar ise her zaman gorunur ve aralari sabittir; hangisi
        oldugu _yol_kenarlari icinde yol genisligine gore cozulur.
        Sari gorundugunde dogrulama icin kullanilir.
        """
        b_nokta = self._yol_kenarlari(kb_beyaz)
        if b_nokta:
            self.beklenen_beyaz = float(np.clip(
                0.7 * self.beklenen_beyaz + 0.3 * b_nokta[0][1], -6.0, 2.0))

        s_nokta = self._cizgi_noktalari(kb_sari, self.beklenen_sari)
        if s_nokta:
            self.beklenen_sari = float(np.clip(
                0.7 * self.beklenen_sari + 0.3 * s_nokta[0][1], -3.0, 6.0))
        elif b_nokta:
            # sari kaybolduysa beklentisini dis kenardan tasi
            self.beklenen_sari = self.beklenen_beyaz + self.serit_w

        b = dict((round(X, 2), Y) for X, Y in b_nokta)
        s = dict((round(X, 2), Y) for X, Y in s_nokta)
        yari = self.serit_w / 2.0

        merkez = []
        for X in sorted(set(b) | set(s)):
            if X in b and X in s:
                # Tutarlilik: sari, beyazin tam bir serit solunda olmali.
                # Degilse beyaz yanlis cizgidir; sariya guven.
                ayrim = s[X] - b[X]
                if 0.6 * self.serit_w < ayrim < 1.5 * self.serit_w:
                    merkez.append((X, (b[X] + s[X]) / 2.0))
                else:
                    merkez.append((X, s[X] - yari))
            elif X in s:                      # sadece sari orta cizgi
                merkez.append((X, s[X] - yari))
            else:                             # sadece beyaz kenar
                merkez.append((X, b[X] + yari))
        return merkez, b_nokta, s_nokta

    # ================= kontrol =================
    def _saf_takip(self, merkez, hiz):
        """Saf takip: on bakis noktasina goturen direksiyon acisi.

            delta = atan( 2 L sin(alpha) / Ld )

        Dogrudan direksiyon acisi uretir; ackermann_drive eklentisi
        cmd_vel.angular.z'yi tam da boyle yorumluyor (olcumle dogrulandi).
        """
        Ld = float(np.clip(self.ob_kat * max(hiz, 1.0),
                           self.ob_min, self.ob_maks))
        hedef = None
        for X, Y in merkez:
            if X >= Ld:
                hedef = (X, Y)
                break
        if hedef is None and merkez:
            hedef = merkez[-1]          # en uzak nokta
        if hedef is None:
            return 0.0, Ld, None

        X, Y = hedef
        # Engelden kacinma: guvenlik katmani sol seridin bos oldugunu
        # dogrulayinca yanal ofset komutu yolluyor. Hedef noktayi bu
        # kadar sola (pozitif Y) kaydirinca araç seridin yanina gecip
        # engeli siyiriyor. Ofset guvenlik tarafinda rampalaniyor.
        Y = Y + self.kacinma_ofset
        mesafe = math.hypot(X, Y)
        if mesafe < 0.5:
            return 0.0, Ld, (X, Y)
        alpha = math.atan2(Y, X)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), mesafe)
        return (float(np.clip(delta, -self.maks_dir, self.maks_dir)),
                Ld, (X, Y))

    @staticmethod
    def _egrilik(merkez):
        """Merkez hattina 2. derece polinom uydurup egrilik cikarir."""
        if len(merkez) < 6:
            return 0.0
        X = np.array([p[0] for p in merkez])
        Y = np.array([p[1] for p in merkez])
        try:
            a = np.polyfit(X, Y, 2)[0]
        except (np.linalg.LinAlgError, ValueError):
            return 0.0
        return float(2.0 * a)          # y = a x^2 -> y'' = 2a

    # ================= ana dongu =================
    def goruntu_cb(self, msg):
        kare = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = kare.shape[:2]

        if self.ipm is None:
            fov, kh, ki, ke = self._ipm_params
            self.ipm = IPM(w, h, fov, kh, ki, ke,
                           x_min=5.0, x_max=25.0, y_yari=6.0)
            self.get_logger().info(
                f'IPM kuruldu: {w}x{h} -> kus bakisi '
                f'{self.ipm.kb_g}x{self.ipm.kb_y}')

        twist = Twist()
        komut_doldu = self._simdi() >= self.komut_biter

        if self.mevcut_komut == 'dur':
            if komut_doldu:
                self.mevcut_komut = 'ileri'
                self.hiz_limiti = self.hiz_duz
            self.cmd_pub.publish(twist)      # sifir = dur
            self._debug(kare, None, None, None, None, 'DUR')
            return

        # ---- sagdon manevrasi aktifse: scripted U-donusu ----
        if self.sagdon_faz > 0:
            # Faz 7'de (yola donus) serit bulunursa manevravi bitir
            if self.sagdon_faz == 7:
                beyaz_t, sari_t = self._maskeler(kare)
                kb_b = self.ipm.cevir(beyaz_t)
                kb_s = self.ipm.cevir(sari_t)
                merkez_t, _, _ = self._serit_merkezi(kb_b, kb_s)
                if len(merkez_t) >= 6:
                    self.sagdon_faz = 0
                    self.get_logger().info(
                        'SAGA DON: serit bulundu, normal takibe donus')
                    # normal lane following devam edecek asagida
                else:
                    tick = self._sagdon_tick()
                    if tick is not None:
                        twist.linear.x = float(tick[0])
                        twist.angular.z = float(tick[1])
                    self.cmd_pub.publish(twist)
                    self._debug(kare, kb_b, kb_s, None, None,
                                f'SAGDON donus faz=7 '
                                f'yaw={math.degrees(self.arac_yaw):.0f}°')
                    return
            else:
                tick = self._sagdon_tick()
                if tick is not None:
                    twist.linear.x = float(tick[0])
                    twist.angular.z = float(tick[1])
                    self.cmd_pub.publish(twist)
                    self._debug(kare, None, None, None, None,
                                f'SAGDON faz={self.sagdon_faz} '
                                f'yaw={math.degrees(self.arac_yaw):.0f}°')
                    return
                # tick None -> manevra bitti, serit takibine devam et

        beyaz, sari = self._maskeler(kare)
        kb_beyaz = self.ipm.cevir(beyaz)
        kb_sari = self.ipm.cevir(sari)

        # ---- yaya gecidi: serit icinde beyaz blok ----
        # Tek kotu kare (orn. virajda kenar cizgisinin ROI'ye girmesi)
        # yanlis tetiklemesin diye YAYA_ONAY kadar ardisik kare sart.
        yaya_oran = self._yaya_gecidi_orani(kb_beyaz)
        if yaya_oran > self.YAYA_ESIK:
            self.yaya_sayac += 1
        else:
            self.yaya_sayac = 0
        if (self.yaya_sayac >= self.YAYA_ONAY
                and self._tetiklenebilir('yaya_gecidi')):
            self.hiz_limiti = self.YAYA_HIZ
            self.yaya_biter = self._simdi() + self.YAYA_SURE
            self.get_logger().info(
                f'YAYA GECIDI! oran={yaya_oran:.2f}, yavasladi')
        # yaya gecidi hiz kisitlamasi suresi dolunca normale don
        if self.hiz_limiti == self.YAYA_HIZ and self._simdi() > self.yaya_biter:
            self.hiz_limiti = self.hiz_duz

        merkez, b_nokta, s_nokta = self._serit_merkezi(kb_beyaz, kb_sari)

        # ---- serit kaybi ----
        if len(merkez) < 4:
            self.kayip_sayac += 1
            if self.kacinma_ofset > 0.1:
                twist.linear.x = self.hiz_viraj * 0.6
                twist.angular.z = 0.0
                durum = f'SERIT YOK - KACINMA ({self.kayip_sayac})'
            elif self.kayip_sayac >= self.KAYIP_DUR:
                twist.linear.x = 0.0
                durum = 'SERIT YOK - DURDU'
            else:
                # son bilinen direksiyonu koru, yavasla
                oran = max(0.0, 1.0 - self.kayip_sayac / self.KAYIP_DUR)
                twist.linear.x = self.hiz_viraj * oran
                twist.angular.z = self.son_direksiyon
                durum = f'SERIT ZAYIF ({self.kayip_sayac})'
            self.cmd_pub.publish(twist)
            self._debug(kare, kb_beyaz, kb_sari, b_nokta, s_nokta, durum)
            return

        self.kayip_sayac = 0

        # ---- egrilige gore hiz ----
        egri = self._egrilik(merkez)
        self.suzulmus_egri = 0.8 * self.suzulmus_egri + 0.2 * egri
        viraj = min(1.0, abs(self.suzulmus_egri) / 0.02)
        hedef_hiz = self.hiz_duz + (self.hiz_viraj - self.hiz_duz) * viraj
        hedef_hiz = min(hedef_hiz, self.hiz_limiti)

        delta, Ld, hedef = self._saf_takip(merkez, max(self.arac_hiz, hedef_hiz))
        self.son_direksiyon = delta

        twist.linear.x = float(hedef_hiz)
        twist.angular.z = float(delta)
        self.cmd_pub.publish(twist)

        self._debug(kare, kb_beyaz, kb_sari, b_nokta, s_nokta,
                    f'v={hedef_hiz:.1f} d={math.degrees(delta):+.1f}deg '
                    f'yaya={yaya_oran:.2f}',
                    merkez, hedef)

    # ================= hata ayiklama goruntusu =================
    def _debug(self, kare, kb_beyaz, kb_sari, b_nokta, s_nokta, durum,
               merkez=None, hedef=None):
        if not self.debug_yayinla:
            return
        if kb_beyaz is None:
            gorsel = kare.copy()
        else:
            gorsel = np.zeros((self.ipm.kb_y, self.ipm.kb_g, 3), np.uint8)
            gorsel[kb_beyaz > 0] = (255, 255, 255)
            gorsel[kb_sari > 0] = (0, 255, 255)

            def ciz(noktalar, renk, kalin=2):
                for X, Y in noktalar or []:
                    px = int(self.ipm.y_to_kb_sutun(Y))
                    py = int((self.ipm.x_max - X) / self.ipm.cozunurluk)
                    if 0 <= px < self.ipm.kb_g and 0 <= py < self.ipm.kb_y:
                        cv2.circle(gorsel, (px, py), kalin, renk, -1)

            ciz(b_nokta, (180, 180, 180))
            ciz(s_nokta, (0, 200, 200))
            ciz(merkez, (0, 0, 255), 2)
            if hedef:
                px = int(self.ipm.y_to_kb_sutun(hedef[1]))
                py = int((self.ipm.x_max - hedef[0]) / self.ipm.cozunurluk)
                cv2.circle(gorsel, (px, py), 7, (0, 255, 0), 2)
            # araç ekseni
            orta = int(self.ipm.y_to_kb_sutun(0.0))
            cv2.line(gorsel, (orta, 0), (orta, self.ipm.kb_y), (255, 0, 0), 1)

        cv2.putText(gorsel, durum, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 255), 1, cv2.LINE_AA)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(gorsel, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    dugum = SeritTakip()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
