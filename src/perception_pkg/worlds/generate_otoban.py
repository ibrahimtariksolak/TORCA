#!/usr/bin/env python3
"""Otoban dunyasini parametrik olarak uretir.

Neden script? Onceki otoban.world Gazebo GUI'sinden kaydedilmisti ve iki
problemi vardi:
  1. GUI'nin yazdigi <state> blogu model pozlarini eziyordu; sol/sag serit
     cizgileri farkli uzunluk ve merkeze kayiyordu (scale 1.228 vs 1.474).
  2. Pist 80 m duz cizgiydi; serit takibi duz yolda gozlemlenemez.

Bu script bunun yerine kapali bir oval devre uretir: iki duz bolum + iki
180 derece viraj. Araç devreyi suresiz donebilir, viraj serit takibini
gercekten sinar.

Kullanim:
    python3 generate_otoban.py            # otoban.world dosyasina yazar
    python3 generate_otoban.py --help     # parametreler
"""

import argparse
import math

# --- Geometri parametreleri (metre) ---------------------------------------
DUZ_UZUNLUK = 60.0      # her duz bolumun uzunlugu
VIRAJ_YARICAP = 25.0    # orta hat viraj yaricapi
SERIT_GENISLIK = 3.5    # tek serit genisligi -> yol 2 x 3.5 = 7 m
YOL_PAYI = 0.75         # asfaltin kenar cizgisinden tasma miktari
CIZGI_GENISLIK = 0.15   # yol cizgisi kalinligi
CIZGI_Z = 0.012         # cizgilerin zemin ustu yuksekligi
YOL_Z = 0.005           # asfalt seridinin zemin ustu yuksekligi

# Kesikli orta cizgi
KESIK_BOY = 3.0
KESIK_ARA = 6.0

# Ornekleme adimlari
CIZGI_ADIM = 1.0        # duz cizgi segment boyu
YOL_ADIM = 2.0          # asfalt segment boyu; kisa adim viraj kenarini duzler
BARIYER_ADIM = 4.0
BARIYER_YUKSEK = 0.8
BARIYER_KALIN = 0.3

# Tabela mesh olcegi. 1.0 (gercek boyut) ile YOLO tabelayi ancak
# 10 m'den yakinda taniyordu; bkz. tools/tabela_kalibre.py
TABELA_OLCEK = 2.5


def merkez_hat_uzunluk(duz, yaricap):
    """Oval orta hattin toplam uzunlugu."""
    return 2.0 * duz + 2.0 * math.pi * yaricap


def merkez_hat_nokta(s, duz, yaricap):
    """Orta hat uzerinde s mesafesindeki (x, y, teta) degerini dondurur.

    Oval su sirayla gezilir (saat yonunun tersi):
      1. Alt duz:   (-duz/2, -R) -> (+duz/2, -R), yon +x
      2. Sag viraj: merkez (+duz/2, 0), aci -90 -> +90
      3. Ust duz:   (+duz/2, +R) -> (-duz/2, +R), yon -x
      4. Sol viraj: merkez (-duz/2, 0), aci +90 -> +270
    """
    viraj_boy = math.pi * yaricap
    toplam = 2.0 * duz + 2.0 * viraj_boy
    s = s % toplam

    yarim = duz / 2.0

    # 1. alt duz
    if s < duz:
        return (-yarim + s, -yaricap, 0.0)
    s -= duz

    # 2. sag viraj
    if s < viraj_boy:
        aci = -math.pi / 2.0 + s / yaricap
        return (yarim + yaricap * math.cos(aci),
                yaricap * math.sin(aci),
                aci + math.pi / 2.0)
    s -= viraj_boy

    # 3. ust duz
    if s < duz:
        return (yarim - s, yaricap, math.pi)
    s -= duz

    # 4. sol viraj
    aci = math.pi / 2.0 + s / yaricap
    return (-yarim + yaricap * math.cos(aci),
            yaricap * math.sin(aci),
            aci + math.pi / 2.0)


def ofset_nokta(s, ofset, duz, yaricap):
    """Orta hattan `ofset` kadar yana kaydirilmis nokta.

    Pozitif ofset sola (orta hattin normal yonu), negatif saga kaydirir.
    Viraj icinde yaricap otomatik duzeltilir: sola kayma dis viraji
    buyutur, ic viraji kucultur. Bu yuzden ark boyu da degisir -- bu
    fonksiyon konum dondurur, ark boyu hesabi cagirana aittir.
    """
    x, y, teta = merkez_hat_nokta(s, duz, yaricap)
    nx = -math.sin(teta)
    ny = math.cos(teta)
    return (x + ofset * nx, y + ofset * ny, teta)


def ofset_hat_uzunluk(ofset, duz, yaricap):
    """Ofsetlenmis hattin toplam uzunlugu.

    Duz bolumler degismez; viraj yaricapi (yaricap - ofset) olur cunku
    sola (+) kayma oval merkezine dogru gider -> ic viraj.
    """
    return 2.0 * duz + 2.0 * math.pi * (yaricap - ofset)


def ofset_hat_nokta(t, ofset, duz, yaricap):
    """Ofsetlenmis hat uzerinde t mesafesindeki (x, y, teta).

    t, ofsetlenmis hattin kendi yay uzunlugudur; duzgun aralikli
    segmentler icin bu gerekir (orta hat s'si ile dogrudan oranli degil).
    """
    r = yaricap - ofset
    viraj_boy = math.pi * r
    toplam = 2.0 * duz + 2.0 * viraj_boy
    t = t % toplam
    yarim = duz / 2.0

    if t < duz:
        return (-yarim + t, -yaricap + ofset, 0.0)
    t -= duz

    if t < viraj_boy:
        aci = -math.pi / 2.0 + t / r
        return (yarim + r * math.cos(aci), r * math.sin(aci),
                aci + math.pi / 2.0)
    t -= viraj_boy

    if t < duz:
        return (yarim - t, yaricap - ofset, math.pi)
    t -= duz

    aci = math.pi / 2.0 + t / r
    return (-yarim + r * math.cos(aci), r * math.sin(aci),
            aci + math.pi / 2.0)


# --- SDF uretimi ----------------------------------------------------------

BEYAZ = """<material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0.25 0.25 0.25 1</emissive>
          </material>"""

SARI = """<material>
            <ambient>0.85 0.75 0.1 1</ambient>
            <diffuse>1 0.85 0.1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0.25 0.2 0.0 1</emissive>
          </material>"""

ASFALT = """<material>
            <script>
              <uri>model://asphalt_plane/materials/scripts</uri>
              <uri>model://asphalt_plane/materials/textures</uri>
              <name>vrc/asphalt</name>
            </script>
          </material>"""

BARIYER_MAT = """<material>
            <ambient>0.75 0.75 0.78 1</ambient>
            <diffuse>0.8 0.8 0.85 1</diffuse>
          </material>"""


def visual(ad, poz, boyut, malzeme):
    x, y, z, yaw = poz
    sx, sy, sz = boyut
    return f"""        <visual name='{ad}'>
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {malzeme}
        </visual>
"""


def collision(ad, poz, boyut):
    x, y, z, yaw = poz
    sx, sy, sz = boyut
    return f"""        <collision name='{ad}'>
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
"""


def surekli_hat(ad, ofset, genislik, malzeme, duz, yaricap,
                adim=CIZGI_ADIM, z=CIZGI_Z, kalinlik=0.01):
    """Kesintisiz cizgiyi tek model / tek link icinde N visual olarak uretir.

    Hepsi tek link oldugu icin Gazebo yuzlerce ayri model yerine bir tek
    statik govde isler -- GUI'den surukleyerek bunu yapmak imkansiz.
    """
    toplam = ofset_hat_uzunluk(ofset, duz, yaricap)
    n = max(1, int(round(toplam / adim)))
    ds = toplam / n
    # %5 bindirme: viraj icinde segmentler arasi bosluk kalmasin
    boy = ds * 1.05

    parcalar = []
    for i in range(n):
        t = (i + 0.5) * ds
        x, y, teta = ofset_hat_nokta(t, ofset, duz, yaricap)
        parcalar.append(visual(f'v{i}', (x, y, z, teta),
                               (boy, genislik, kalinlik), malzeme))

    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
{''.join(parcalar)}      </link>
    </model>
"""


def kesikli_hat(ad, ofset, genislik, malzeme, duz, yaricap,
                kesik=KESIK_BOY, ara=KESIK_ARA, z=CIZGI_Z, kalinlik=0.01):
    """Kesikli orta cizgi. Her kesik, viraji takip etmesi icin 1 m'lik
    alt parcalara bolunur."""
    toplam = ofset_hat_uzunluk(ofset, duz, yaricap)
    periyot = kesik + ara
    n_kesik = max(1, int(toplam / periyot))
    # Periyodu tam bolunecek sekilde ayarla -> devrenin basi ve sonu eslesir
    periyot = toplam / n_kesik
    kesik = periyot * (kesik / (kesik + ara))

    parcalar = []
    idx = 0
    for k in range(n_kesik):
        bas = k * periyot
        alt_n = max(1, int(round(kesik / CIZGI_ADIM)))
        ds = kesik / alt_n
        boy = ds * 1.05
        for j in range(alt_n):
            t = bas + (j + 0.5) * ds
            x, y, teta = ofset_hat_nokta(t, ofset, duz, yaricap)
            parcalar.append(visual(f'v{idx}', (x, y, z, teta),
                                   (boy, genislik, kalinlik), malzeme))
            idx += 1

    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
{''.join(parcalar)}      </link>
    </model>
"""


def yol_yuzeyi(ad, duz, yaricap, yarim_genislik, adim=YOL_ADIM, z=YOL_Z):
    """Asfalt seridi -- sadece visual. Carpisma zemin duzleminden gelir,
    boylece yuzlerce collision govdesi fizik motorunu yormaz.

    Bindirme orani sabit OLAMAZ. Segmentler orta hatta gore yerlestirilir
    ama viraj icinde dis kenarin yayi orta hattinkinden uzundur:
        dis yay / orta yay = (R + yarim_genislik) / R
    Sabit %6 bindirme ile 8.5 m genisliginde bir yol, R=25 virajda dis
    kenarda 0.44 m'lik bosluklar birakiyordu. Bindirmeyi bu orandan
    turetiyoruz."""
    toplam = merkez_hat_uzunluk(duz, yaricap)
    n = max(1, int(round(toplam / adim)))
    ds = toplam / n
    # dis kenari kapatacak bindirme + %3 emniyet payi
    bindirme = (yaricap + yarim_genislik) / yaricap * 1.03
    boy = ds * bindirme
    en = 2.0 * yarim_genislik

    parcalar = []
    for i in range(n):
        s = (i + 0.5) * ds
        x, y, teta = merkez_hat_nokta(s, duz, yaricap)
        parcalar.append(visual(f'v{i}', (x, y, z, teta),
                               (boy, en, 0.01), ASFALT))

    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
{''.join(parcalar)}      </link>
    </model>
"""


def bariyer(ad, ofset, duz, yaricap, adim=BARIYER_ADIM,
            yukseklik=BARIYER_YUKSEK, kalinlik=BARIYER_KALIN):
    """Yol kenari bariyeri -- visual + collision (araci pistte tutar)."""
    toplam = ofset_hat_uzunluk(ofset, duz, yaricap)
    n = max(1, int(round(toplam / adim)))
    ds = toplam / n
    boy = ds * 1.02
    z = yukseklik / 2.0

    vis, col = [], []
    for i in range(n):
        t = (i + 0.5) * ds
        x, y, teta = ofset_hat_nokta(t, ofset, duz, yaricap)
        poz = (x, y, z, teta)
        boyut = (boy, kalinlik, yukseklik)
        vis.append(visual(f'v{i}', poz, boyut, BARIYER_MAT))
        col.append(collision(f'c{i}', poz, boyut))

    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
{''.join(col)}{''.join(vis)}      </link>
    </model>
"""


def tabela(ad, model_uri, mesh_adi, malzeme_adi, s, ofset, duz, yaricap,
           olcek=TABELA_OLCEK, yaw_ek=0.0):
    """Hazir tabela mesh'ini pistin `s` mesafesinde, `ofset` yaninda koyar.

    Tabela yuzu araca donuk olacak sekilde dondurulur (yaw + 180 derece,
    cunku araç +t yonunde gelir ve tabelaya onden bakar).

    `olcek` neden gerekli: YOLO'nun bir tabelayi siniflandirabilmesi icin
    goruntude yeterince buyuk olmasi lazim. Olcumde gercek boyutlu dur
    tabelasi ancak 10 m'den yakinda taninabiliyordu (4 m/s'de 2.5 s
    tepki suresi). fy=380 ile 25 m'de 28 px bbox elde etmek icin tabela
    ~1.8 m yuksekliginde olmali.
    """
    x, y, teta = ofset_nokta(s, ofset, duz, yaricap)
    # Tabela yuzu YOLA DIK olmali, yani gelen araca donuk.
    #
    # Ilk surumde yaw = teta + pi kullaniliyordu ve tabelalar yola
    # PARALEL duruyordu: araç onlari kenardan, ince bir cizgi olarak
    # goruyordu. YOLO hicbir sey taniyamiyordu.
    #
    # Olcum (scratchpad/yaw_testi.py, kamera +x yonune bakarken):
    #   yaw=0      -> kenardan, ince cizgi
    #   yaw=pi/2   -> tabelanin ARKASI (beyaz sekizgen)
    #   yaw=pi     -> kenardan, ince cizgi   <- eski hatali deger
    #   yaw=3pi/2  -> YUZU (kirmizi sekizgen + STOP yazisi)  <- dogru
    # Yani yol tegeti teta iken dogru yaw = teta - pi/2.
    yaw = teta - math.pi / 2.0 + yaw_ek

    # Malzeme bloğu: modelin materials/scripts dizini varsa ZORUNLU.
    # Ilk denemede atlanmisti ve mesh dokusuz gri bir sekil olarak
    # render edildi; dur tabelasinin kirmizi sekizgeni ve STOP yazisi
    # dokuda oldugundan YOLO tespiti tamamen sifira dustu.
    # speed_limit_sign'da scripts dizini yok, dokuyu .dae kendi
    # tasiyor; orada blok atlanmali.
    malzeme = '' if malzeme_adi is None else f"""
          <material>
            <script>
              <uri>model://{model_uri}/materials/scripts</uri>
              <uri>model://{model_uri}/materials/textures</uri>
              <name>{malzeme_adi}</name>
            </script>
          </material>"""

    # <include> SDF'te olcek desteklemiyor, bu yuzden model inline
    # kuruluyor ve mesh <scale> ile buyutuluyor.
    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.5f}</pose>
      <link name='link'>
        <visual name='v'>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <mesh>
              <uri>model://{model_uri}/meshes/{mesh_adi}</uri>
              <scale>{olcek} {olcek} {olcek}</scale>
            </mesh>
          </geometry>{malzeme}
        </visual>
      </link>
    </model>
"""


def engel(ad, s, ofset, duz, yaricap,
          genislik=1.2, derinlik=0.5, yukseklik=1.0):
    """Pistin ileri bir noktasina duran bir engel koyar.

    Kullanim: LiDAR tabanli acil fren katmanini sinamak icin. Engel
    aracin yoluna duser ve kontrolcu seritten disari cikmadan durmak
    zorunda kalir. Statik, kirmizi, carpisma geometrisi var.
    """
    x, y, _ = ofset_nokta(s, ofset, duz, yaricap)
    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} {yukseklik/2:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='c'>
          <geometry>
            <box><size>{derinlik:.3f} {genislik:.3f} {yukseklik:.3f}</size></box>
          </geometry>
        </collision>
        <visual name='v'>
          <geometry>
            <box><size>{derinlik:.3f} {genislik:.3f} {yukseklik:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.9 0.15 0.15 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def yaya_gecidi(ad, s_merkez, duz, yaricap, serit,
                serit_en=0.45, bosluk=0.22, uzunluk=4.5):
    """Yola zebra seritleri cizer (yaya gecidi).

    Standart zebra: beyaz bantlar araç YONUNE PARALEL uzanir (her biri
    `uzunluk` m) ve yol boyunca YANAL olarak dizilir. Surucu yaklasirken
    bir dizi beyaz cubuk gorur.

    Bantlar sag seridin genisligini kaplar ama kenar cizgilerine
    degmez. s_merkez gecidin orta hat uzerindeki konumu; bantlar bunun
    etrafina `uzunluk`/2 kadar yayilir.
    """
    parcalar = []
    # Sag serit: orta hattan -serit'e kadar. Zebra bu bandi doldursun.
    # Kenar cizgilerine degmemesi icin kucuk pay birak.
    ic = -0.08 * serit           # orta cizgiye yakin kenar
    dis = -0.94 * serit          # dis kenar cizgisine yakin
    n = max(2, int(abs(dis - ic) / (serit_en + bosluk)))
    idx = 0
    for i in range(n):
        yan = ic + (i + 0.5) * (dis - ic) / n
        # Her bant, travel yonunde `uzunluk` m: kus bakisinda net blok
        alt_n = max(1, int(round(uzunluk / YOL_ADIM)))
        for j in range(alt_n):
            ds = uzunluk / alt_n
            s = s_merkez - uzunluk / 2 + (j + 0.5) * ds
            x, y, teta = ofset_nokta(s, yan, duz, yaricap)
            parcalar.append(visual(f'z{idx}', (x, y, CIZGI_Z, teta),
                                   (ds * 1.05, serit_en, 0.01), BEYAZ))
            idx += 1
    return f"""    <model name='{ad}'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
{''.join(parcalar)}      </link>
    </model>
"""


def trafik_araci(ad, renk, s, ofset, duz, yaricap,
                 uzun=4.0, en=1.8, yuk=1.4):
    """Hareketli trafik araci: statik DEGIL, carpisma govdesi var.

    Konumu perception_pkg/trafik.py dugumu /gazebo/set_entity_state ile
    her karede gunceller. Statik olmadigi icin LiDAR onu goruyor ve
    guvenlik katmani dogal olarak tepki veriyor.

    Kutle yuksek ve atalet buyuk secildi: set_entity_state ile
    tasindiklari icin fizik onlari surukleyemesin.

    `s` OFSET HAT yay uzunlugudur (orta hat degil); trafik.py de ayni
    parametrelemeyi kullaniyor, boylece baslangic pozu ile calisma
    zamani pozu ayni noktaya denk geliyor.
    """
    x, y, teta = ofset_hat_nokta(s, ofset, duz, yaricap)
    r, g, b = renk
    return f"""    <model name='{ad}'>
      <static>0</static>
      <pose>{x:.4f} {y:.4f} {yuk/2:.3f} 0 0 {teta:.5f}</pose>
      <link name='govde'>
        <inertial>
          <mass>1500.0</mass>
          <inertia>
            <ixx>800</ixx><iyy>2200</iyy><izz>2500</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name='c'>
          <geometry>
            <box><size>{uzun:.2f} {en:.2f} {yuk:.2f}</size></box>
          </geometry>
        </collision>
        <visual name='v'>
          <geometry>
            <box><size>{uzun:.2f} {en:.2f} {yuk:.2f}</size></box>
          </geometry>
          <material>
            <ambient>{r*0.6:.2f} {g*0.6:.2f} {b*0.6:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          </material>
        </visual>
        <visual name='cam'>
          <pose>-0.3 0 {yuk*0.33:.2f} 0 0 0</pose>
          <geometry>
            <box><size>{uzun*0.42:.2f} {en*0.92:.2f} {yuk*0.42:.2f}</size></box>
          </geometry>
          <material>
            <ambient>0.1 0.12 0.16 1</ambient>
            <diffuse>0.15 0.18 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


# Trafik araclari icin ayirt edici renkler (sunumda net gorunsun)
TRAFIK_RENKLERI = [
    (0.85, 0.15, 0.15),   # kirmizi
    (0.15, 0.55, 0.85),   # mavi
    (0.95, 0.75, 0.10),   # sari
    (0.20, 0.70, 0.30),   # yesil
    (0.85, 0.45, 0.10),   # turuncu
    (0.60, 0.25, 0.75),   # mor
]


def ozel_tabela(ad, model, s, ofset, duz, yaricap, yaw_ek=0.0):
    """tools/tabela_model_uret.py'nin urettigi tabela_* modellerini koyar.

    tabela() stop_sign.dae / speed_limit_sign.dae gibi hazir mesh'leri
    kullanir ve onlarin yuz normali farkli (yaw = teta - pi/2).

    Bu fonksiyondaki ozel tabela modellerinde LEVHA kutusu X normalli
    (bkz. tabela_model_uret.py: box size KALIN BOYUT BOYUT). Yaw=0 iken
    yuz +X'e bakar. Araç +s yonunden gelirken yola DIK ve araca donuk
    olmasi icin:
        yaw = teta + pi   (yuz dunyada -t yonune, araç geldigi yone)
    Bu veri_topla.py'de spawn ederken kullandigimiz yaw=pi ile tutarli
    (orada teta=0'di).

    `model` ornek: 'tabela_giris_yok', 'tabela_yaya_mavi'.
    """
    x, y, teta = ofset_nokta(s, ofset, duz, yaricap)
    yaw = teta + math.pi + yaw_ek
    return f"""    <include>
      <uri>model://{model}</uri>
      <name>{ad}</name>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.5f}</pose>
    </include>
"""


def dunya_uret(duz=DUZ_UZUNLUK, yaricap=VIRAJ_YARICAP,
               serit=SERIT_GENISLIK, golgeler=False, bariyerli=True,
               engel_s=None, yaya_s=None, trafik_sayisi=0,
               trafik_bas_s=60.0, showcase=False):
    kenar = serit                       # orta hattan kenar cizgisine
    yarim_yol = serit + YOL_PAYI        # asfalt yarim genisligi

    # Pistin kapladigi alan -> zemin duzlemi boyutu
    span_x = duz + 2.0 * (yaricap + yarim_yol) + 20.0
    span_y = 2.0 * (yaricap + yarim_yol) + 20.0
    zemin = max(span_x, span_y)

    merkez_boy = merkez_hat_uzunluk(duz, yaricap)

    parcalar = []

    # --- asfalt seridi
    parcalar.append(yol_yuzeyi('yol_yuzeyi', duz, yaricap, yarim_yol))

    # --- cizgiler: dis kenar (sag, -ofset), ic kenar (sol, +ofset)
    parcalar.append(surekli_hat('dis_kenar_cizgi', -kenar, CIZGI_GENISLIK,
                                BEYAZ, duz, yaricap))
    parcalar.append(surekli_hat('ic_kenar_cizgi', +kenar, CIZGI_GENISLIK,
                                BEYAZ, duz, yaricap))
    parcalar.append(kesikli_hat('orta_cizgi', 0.0, CIZGI_GENISLIK,
                                SARI, duz, yaricap))

    # --- bariyerler (asfalt payinin hemen disinda)
    if bariyerli:
        parcalar.append(bariyer('dis_bariyer', -(yarim_yol + 0.5),
                                duz, yaricap))
        parcalar.append(bariyer('ic_bariyer', +(yarim_yol + 0.5),
                                duz, yaricap))

    # --- tabelalar: araç saat yonu tersine gider ve sag serittedir,
    #     dolayisiyla tabelalar yolun sag kenarinin disina konur.
    #
    # Yerlesim keyfi degil, pistin yapisina gore secildi. Oval uzerinde
    # s = 0, 60, 138.5, 198.5 noktalari sirasiyla alt duzun basi, sag
    # virajin girisi, ust duzun basi ve sol virajin girisidir.
    #
    # NOT: Elde YOLO'nun GERCEKTEN tanidigi tek model stop_sign.
    # speed_limit_sign kalibrasyonda hic tespit edilmedi (model gercek
    # tabela fotograflariyla egitilmis, bu mesh'i tabela olarak
    # gormuyor); gorsel zenginlik icin duruyor ama kontrolcu ona tepki
    # veremiyor. Bu yuzden TANINAN tabela sayisini artirmanin tek yolu
    # stop_sign'i pistin farkli noktalarina dagitmak.
    tabela_ofset = -(kenar + 1.2)
    viraj_boy = math.pi * yaricap

    # Pist uzerindeki onemli noktalar (orta hat s cinsinden):
    #   0            alt duzun basi (araç s=12'de doguyor, +s yonune gider)
    #   duz          sag virajin girisi
    #   duz+vb       ust duzun basi
    #   2*duz+vb     sol virajin girisi
    #   2*(duz+vb)   tur tamamlandi
    #
    # Tabelalar surucunun karar vermesi gereken yerlere dagitiliyor:
    # viraj girislerinden once ve duz bolumler boyunca.
    #
    # (ad, s, tip) -- tip 'dur' YOLO tarafindan taninir, 'hiz' gorseldir
    if showcase:
        # Showcase: fine-tune edilmis modelin tanidigi TUM sinif
        # ailelerini pist boyunca dagit, karar aralarini genis tut
        # (DUR 3 s, GIRIS YOK 5 s, YAYA 4 s yavaslama).
        # Showcase yerlesimi: fine-tune ile taninan 5 sinif ailesini
        # pist boyunca GENIS araliklarla dagit. YOL VER ve GIRIS YOK
        # dahil edilmedi: YOL VER hizi kalici dusuruyor (TABELA_BEKLEME
        # dolana kadar reset olmuyor), GIRIS YOK 5 s durusunu YAYA
        # yavaslamasiyla ust uste getiriyor ve demo akicilgi bozuluyor.
        # dur ve yaya her tur bir kez tetiklenecek; donel ve hiz_30
        # sadece algilanacak (davranis yok, gorsel).
        yerlesim = [
            ('tabela_hiz_30',     12.0,                        'ozel'),
            ('tabela_yaya_mavi',  30.0,                        'ozel'),
            ('tabela_donel',      duz + viraj_boy / 2.0,       'ozel'),
            ('tabela_dur',        duz + viraj_boy + 25.0,      'ozel'),
            ('tabela_saga_don',   2 * duz + viraj_boy - 15.0,  'ozel'),
        ]
    else:
        yerlesim = [
            ('dur_alt_duz',     0.30 * duz,                  'dur'),
            ('hiz_limit_1',     0.60 * duz,                  'hiz'),
            ('dur_viraj1_once', duz - 8.0,                   'dur'),
            ('dur_ust_duz_1',   duz + viraj_boy + 12.0,      'dur'),
            ('hiz_limit_2',     duz + viraj_boy + 30.0,      'hiz'),
            ('dur_ust_duz_2',   duz + viraj_boy + 42.0,      'dur'),
            ('dur_viraj2_once', 2 * duz + viraj_boy - 4.0,   'dur'),
            ('dur_tur_kapanis', 2 * (duz + viraj_boy) - 25.0, 'dur'),
        ]

    # Ayni s'e iki tabela konursa Gazebo'da ust uste binerler ve bu
    # sessizce gozden kacar; ilk surumde dur_viraj1_once ile
    # hiz_limit_1 tam ayni noktaya dusmustu. Uretim sirasinda kontrol:
    toplam_s = 2 * (duz + viraj_boy)
    for i in range(len(yerlesim)):
        for j in range(i + 1, len(yerlesim)):
            fark = abs(yerlesim[i][1] - yerlesim[j][1])
            fark = min(fark, toplam_s - fark)      # dairesel mesafe
            if fark < 10.0:
                raise ValueError(
                    f'{yerlesim[i][0]} ve {yerlesim[j][0]} tabelalari '
                    f'{fark:.1f} m arayla, en az 10 m olmali')

    for ad, s, tip in yerlesim:
        if tip == 'ozel':
            # tabela_* modeli (fine-tune ile taninir); ozel_tabela
            # <include> ile koyar ve yaw'i +X normalli levhaya gore
            # ayarlar
            parcalar.append(ozel_tabela(
                ad.replace('tabela_', ''), ad, s, tabela_ofset,
                duz, yaricap))
            continue
        if tip == 'dur':
            parcalar.append(tabela(ad, 'stop_sign', 'stop_sign.dae',
                                   'StopSign/Diffuse',
                                   s, tabela_ofset, duz, yaricap))
        else:
            # materials/scripts dizini olmadigi icin malzeme adi None,
            # doku .dae icinden geliyor
            parcalar.append(tabela(ad, 'speed_limit_sign',
                                   'speed_limit_sign.dae', None,
                                   s, tabela_ofset, duz, yaricap,
                                   olcek=1.0))

    # --- engel: isteniyorsa pistin s metresine, sag seridin ortasina
    if engel_s is not None:
        parcalar.append(engel('engel', engel_s, -serit / 2.0, duz, yaricap))

    # --- yaya gecidi (zebra seritleri)
    if yaya_s is not None:
        parcalar.append(yaya_gecidi('yaya_gecidi', yaya_s, duz, yaricap,
                                    serit))

    # --- trafik araclari: sag seridin ortasinda, esit aralikli.
    # Konumlarini calisma zamaninda perception_pkg/trafik.py guncelliyor;
    # buradaki pozlar yalnizca baslangic yerlesimi.
    if trafik_sayisi > 0:
        t_ofset = -serit / 2.0
        hat = ofset_hat_uzunluk(t_ofset, duz, yaricap)
        aralik = hat / trafik_sayisi
        for i in range(trafik_sayisi):
            renk = TRAFIK_RENKLERI[i % len(TRAFIK_RENKLERI)]
            parcalar.append(trafik_araci(
                f'trafik_{i+1}', renk,
                (trafik_bas_s + i * aralik) % hat,
                t_ofset, duz, yaricap))

    golge = '1' if golgeler else '0'

    # Acilis kamerasi: tum ovali cerceveleyecek sekilde pistin guney-
    # batisindan bakar. Yoksa Gazebo varsayilan (0,0,0) yakinina bakar ve
    # pist ekranin disinda kalir.
    kam_y = -(yaricap + yarim_yol + 0.6 * span_y)
    kam_z = 0.55 * span_x
    kam_pitch = math.atan2(kam_z, abs(kam_y))
    gui = f"""    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 {kam_y:.2f} {kam_z:.2f} 0 {kam_pitch:.4f} 1.5708</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
"""

    return f"""<?xml version="1.0" ?>
<!-- OTOMATIK URETILDI: generate_otoban.py
     Bu dosyayi elle duzenlemeyin; script'i duzenleyip yeniden calistirin.
     Gazebo GUI'sinden "Save World" YAPMAYIN: GUI bir state blogu yazar
     ve asagidaki model pozlarini sessizce ezer (onceki surumde serit
     cizgileri bu yuzden kaymisti).

     Pist: oval devre, {duz:.0f} m duz x2 + {yaricap:.0f} m yaricapli viraj x2
     Orta hat uzunlugu: {merkez_boy:.1f} m
     Yol: 2 x {serit:.2f} m serit
-->
<sdf version="1.6">
  <world name='otoban'>

    <!-- Fizik: 1 ms adim, 50 iterasyon. Arac temas kararliligi icin
         varsayilan 20 iterasyon yetersiz kaliyor. -->
    <physics name='varsayilan' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- gazebo_ros_state bir WorldPlugin'dir: gzserver'a -s ile
         verilirse "incorrect plugin type" hatasiyla SESSIZCE basarisiz
         olur ve /gazebo/set_entity_state servisi hic olusmaz. Dunya
         dosyasinin icinde tanimlanmak zorunda. (Trafik araclarini
         script'le hareket ettirmek bu servise bagli.) -->
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>50.0</update_rate>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <!-- golgeler kapali: sert golgeler serit tespitini bozuyor.
           Dayaniklilik testi icin golgeler=True ile yeniden uret. -->
      <shadows>{golge}</shadows>
    </scene>

{gui}
    <!-- Gunes tam tepeye yakin: yola dusen govde golgesini azaltir -->
    <light name='gunes' type='directional'>
      <cast_shadows>{golge}</cast_shadows>
      <pose>0 0 80 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.2 0.15 -0.95</direction>
      <attenuation>
        <range>2000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
      </attenuation>
    </light>

    <!-- Zemin: tek carpisma yuzeyi. Asfalt seridi bunun uzerinde
         sadece gorsel olarak durur. -->
    <model name='zemin'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane>
            <normal>0 0 1</normal>
            <size>{zemin:.1f} {zemin:.1f}</size>
          </plane></geometry>
          <surface>
            <friction><ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode></friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>false</cast_shadows>
          <geometry><plane>
            <normal>0 0 1</normal>
            <size>{zemin:.1f} {zemin:.1f}</size>
          </plane></geometry>
          <material>
            <ambient>0.25 0.35 0.2 1</ambient>
            <diffuse>0.3 0.42 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{''.join(parcalar)}
  </world>
</sdf>
"""


def spawn_pozu(duz=DUZ_UZUNLUK, yaricap=VIRAJ_YARICAP, serit=SERIT_GENISLIK):
    """Aracin dogmasi gereken poz: alt duz bolumde, sag seridin ortasinda."""
    s = 0.2 * duz
    ofset = -serit / 2.0
    x, y, teta = ofset_nokta(s, ofset, duz, yaricap)
    return x, y, teta


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-o', '--cikti', default='otoban.world',
                   help='yazilacak dosya (varsayilan: otoban.world)')
    p.add_argument('--duz', type=float, default=DUZ_UZUNLUK,
                   help=f'duz bolum uzunlugu (varsayilan: {DUZ_UZUNLUK})')
    p.add_argument('--yaricap', type=float, default=VIRAJ_YARICAP,
                   help=f'viraj yaricapi (varsayilan: {VIRAJ_YARICAP})')
    p.add_argument('--serit', type=float, default=SERIT_GENISLIK,
                   help=f'serit genisligi (varsayilan: {SERIT_GENISLIK})')
    p.add_argument('--golgeler', action='store_true',
                   help='golgeleri ac (serit tespiti dayaniklilik testi)')
    p.add_argument('--bariyersiz', action='store_true',
                   help='kenar bariyerlerini koyma')
    p.add_argument('--engel-s', type=float, default=None,
                   help='pistin bu s metresine (orta hat uzunlugu boyunca) '
                        'bir engel koy; LiDAR acil fren testi icin')
    p.add_argument('--yaya-s', type=float, default=None,
                   help='bu s metresine yaya gecidi (zebra) ciz')
    p.add_argument('--trafik', type=int, default=0,
                   help='pistte kac hareketli trafik araci olsun '
                        '(perception_pkg trafik dugumu surer)')
    p.add_argument('--trafik-bas-s', type=float, default=60.0,
                   help='ilk trafik aracinin baslangic s degeri')
    p.add_argument('--showcase', action='store_true',
                   help='showcase pisti: fine-tune ile taninan tum tabela '
                        'cesitleri + yaya + engel + trafik (jury sunumu)')
    a = p.parse_args()

    if a.showcase:
        # Tek bayrakla her senaryoyu kurar; argumanlari override et
        if a.yaya_s is None:
            a.yaya_s = 42.0            # yaya tabelasindan 12 m sonra
        if a.engel_s is None:
            # Alt duz bolumun sonu, YAYA zebrasindan 13 m sonra; araç
            # yaya yavaslamasindan cikip hizlanirken engelle karsilasiyor
            # ve kacinma manevrasini duz bolumde (sol serit garantili bos)
            # tetikliyor. Virajda engel ise yarardi: sol-serit bosluk
            # kontrolu yay-tabanli degil.
            a.engel_s = 55.0
        if a.trafik == 0:
            a.trafik = 3
            a.trafik_bas_s = 140.0     # bizim aracin ilerisinde

    sdf = dunya_uret(duz=a.duz, yaricap=a.yaricap, serit=a.serit,
                     golgeler=a.golgeler, bariyerli=not a.bariyersiz,
                     engel_s=a.engel_s, yaya_s=a.yaya_s,
                     trafik_sayisi=a.trafik, trafik_bas_s=a.trafik_bas_s,
                     showcase=a.showcase)

    with open(a.cikti, 'w') as f:
        f.write(sdf)

    x, y, teta = spawn_pozu(a.duz, a.yaricap, a.serit)
    boy = merkez_hat_uzunluk(a.duz, a.yaricap)
    print(f'{a.cikti} yazildi ({len(sdf.splitlines())} satir)')
    print(f'orta hat uzunlugu : {boy:.1f} m')
    print(f'spawn pozu        : -x {x:.3f} -y {y:.3f} -Y {teta:.4f}')


if __name__ == '__main__':
    main()
