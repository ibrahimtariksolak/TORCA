#!/usr/bin/env python3
"""Trafik tabelasi dokularini uretir ve YOLO'nun taniyip tanimadigini olcer.

NEDEN: Elimizdeki YOLOv8 modeli GTSRB benzeri GERCEK tabela
fotograflariyla egitilmis. Gazebo'nun hazir mesh'leri (speed_limit_sign,
speed_limit_30) stilize oldugu icin model onlari tabela olarak
gormuyor; olcumde 0 tespit verdiler. Tek calisan model stop_sign'di.

COZUM: Tabelalari standart geometriye sadik kalarak kendimiz cizip
doku olarak uretmek, sonra bunlari Gazebo'da duzlem mesh'lere
uygulamak.

Bu script once dokulari uretir, sonra YOLO'yu DOGRUDAN uretilen
PNG'ler uzerinde calistirir. Gazebo'ya girmeden once burada
taninmiyorsa, simulasyonda hic taninmaz -- ucuz ve hizli eleme.

Kullanim:
    python3 tools/tabela_uret.py            # uret + test
    python3 tools/tabela_uret.py --sadece-uret
"""

import argparse
import os

import cv2
import numpy as np

# Tabela dokusu cozunurlugu. Yuksek tutuluyor cunku Gazebo bunu
# uzaktan kucultecek; kaynak keskin olmali.
BOYUT = 512

KIRMIZI = (40, 40, 200)      # BGR
MAVI = (160, 70, 20)
BEYAZ = (255, 255, 255)
SIYAH = (20, 20, 20)
SARI = (40, 200, 230)


def _tuval(renk=(255, 255, 255, 0)):
    """Seffaf arka planli RGBA tuval."""
    t = np.zeros((BOYUT, BOYUT, 4), np.uint8)
    t[:] = renk
    return t


def _daire_tabela(ic_cizim, kenar_renk=KIRMIZI, ic_renk=BEYAZ):
    """Kirmizi halkali yuvarlak tabela (hiz limiti, giris yok...)."""
    t = _tuval((0, 0, 0, 0))
    m = BOYUT // 2
    cv2.circle(t, (m, m), int(BOYUT * 0.47), (*kenar_renk, 255), -1,
               cv2.LINE_AA)
    cv2.circle(t, (m, m), int(BOYUT * 0.36), (*ic_renk, 255), -1,
               cv2.LINE_AA)
    ic_cizim(t, m)
    return t


def hiz_limiti(sayi):
    """Beyaz zemin, kalin kirmizi halka, siyah rakam."""
    def ic(t, m):
        yazi = str(sayi)
        olcek = 7.5 if len(yazi) < 3 else 5.5
        kalin = 28
        (w, h), _ = cv2.getTextSize(yazi, cv2.FONT_HERSHEY_DUPLEX,
                                    olcek, kalin)
        cv2.putText(t, yazi, (m - w // 2, m + h // 2),
                    cv2.FONT_HERSHEY_DUPLEX, olcek, (*SIYAH, 255),
                    kalin, cv2.LINE_AA)
    return _daire_tabela(ic)


def giris_yok():
    """Kirmizi daire, ortasinda yatay beyaz bant."""
    t = _tuval((0, 0, 0, 0))
    m = BOYUT // 2
    cv2.circle(t, (m, m), int(BOYUT * 0.47), (*KIRMIZI, 255), -1,
               cv2.LINE_AA)
    y1, y2 = int(BOYUT * 0.43), int(BOYUT * 0.57)
    x1, x2 = int(BOYUT * 0.17), int(BOYUT * 0.83)
    cv2.rectangle(t, (x1, y1), (x2, y2), (*BEYAZ, 255), -1)
    return t


def yol_ver():
    """Ters beyaz ucgen, kalin kirmizi kenar."""
    t = _tuval((0, 0, 0, 0))
    pay = int(BOYUT * 0.04)
    dis = np.array([[pay, pay], [BOYUT - pay, pay],
                    [BOYUT // 2, BOYUT - pay]], np.int32)
    cv2.fillPoly(t, [dis], (*KIRMIZI, 255), cv2.LINE_AA)
    k = int(BOYUT * 0.17)
    ic = np.array([[pay + k, pay + int(k * 0.8)],
                   [BOYUT - pay - k, pay + int(k * 0.8)],
                   [BOYUT // 2, BOYUT - pay - int(k * 1.5)]], np.int32)
    cv2.fillPoly(t, [ic], (*BEYAZ, 255), cv2.LINE_AA)
    return t


def _yaya_figuru(t, cx, cy, olcek, renk):
    """Basit yaya silueti (bas + govde + bacaklar + kol)."""
    r = int(14 * olcek)
    cv2.circle(t, (cx, cy - int(52 * olcek)), r, (*renk, 255), -1,
               cv2.LINE_AA)
    cv2.line(t, (cx, cy - int(36 * olcek)), (cx, cy + int(6 * olcek)),
             (*renk, 255), int(11 * olcek), cv2.LINE_AA)
    # bacaklar
    cv2.line(t, (cx, cy + int(4 * olcek)),
             (cx - int(20 * olcek), cy + int(46 * olcek)),
             (*renk, 255), int(10 * olcek), cv2.LINE_AA)
    cv2.line(t, (cx, cy + int(4 * olcek)),
             (cx + int(18 * olcek), cy + int(48 * olcek)),
             (*renk, 255), int(10 * olcek), cv2.LINE_AA)
    # kol
    cv2.line(t, (cx + int(2 * olcek), cy - int(26 * olcek)),
             (cx + int(24 * olcek), cy + int(2 * olcek)),
             (*renk, 255), int(9 * olcek), cv2.LINE_AA)


def yaya_gecidi_mavi():
    """Mavi kare, beyaz ucgen icinde yaya + zebra (Avrupa tipi)."""
    t = _tuval((0, 0, 0, 0))
    pay = int(BOYUT * 0.05)
    cv2.rectangle(t, (pay, pay), (BOYUT - pay, BOYUT - pay),
                  (*MAVI, 255), -1)
    ucgen = np.array([[BOYUT // 2, int(BOYUT * 0.13)],
                      [int(BOYUT * 0.13), int(BOYUT * 0.84)],
                      [int(BOYUT * 0.87), int(BOYUT * 0.84)]], np.int32)
    cv2.fillPoly(t, [ucgen], (*BEYAZ, 255), cv2.LINE_AA)
    _yaya_figuru(t, BOYUT // 2, int(BOYUT * 0.52), BOYUT / 512.0 * 1.25,
                 SIYAH)
    # zebra seritleri
    for i in range(4):
        x = int(BOYUT * (0.33 + i * 0.09))
        cv2.line(t, (x, int(BOYUT * 0.74)), (x - int(BOYUT * 0.03),
                 int(BOYUT * 0.80)), (*SIYAH, 255), 7, cv2.LINE_AA)
    return t


def yaya_gecidi_ucgen():
    """Ucgen uyari tipi yaya gecidi (Alman/GTSRB tarzi)."""
    t = _tuval((0, 0, 0, 0))
    pay = int(BOYUT * 0.04)
    dis = np.array([[BOYUT // 2, pay], [pay, BOYUT - pay],
                    [BOYUT - pay, BOYUT - pay]], np.int32)
    cv2.fillPoly(t, [dis], (*KIRMIZI, 255), cv2.LINE_AA)
    k = int(BOYUT * 0.15)
    ic = np.array([[BOYUT // 2, pay + int(k * 1.4)],
                   [pay + k, BOYUT - pay - int(k * 0.7)],
                   [BOYUT - pay - k, BOYUT - pay - int(k * 0.7)]], np.int32)
    cv2.fillPoly(t, [ic], (*BEYAZ, 255), cv2.LINE_AA)
    _yaya_figuru(t, BOYUT // 2, int(BOYUT * 0.60), BOYUT / 512.0 * 1.05,
                 SIYAH)
    return t


def donel_kavsak():
    """Mavi daire, uc beyaz dairesel ok."""
    t = _tuval((0, 0, 0, 0))
    m = BOYUT // 2
    cv2.circle(t, (m, m), int(BOYUT * 0.47), (*MAVI, 255), -1, cv2.LINE_AA)
    r = int(BOYUT * 0.24)
    for bas, son in ((20, 100), (140, 220), (260, 340)):
        cv2.ellipse(t, (m, m), (r, r), 0, bas, son, (*BEYAZ, 255), 26,
                    cv2.LINE_AA)
    return t


def sola_don():
    """Mavi daire, beyaz sola ok."""
    t = _tuval((0, 0, 0, 0))
    m = BOYUT // 2
    cv2.circle(t, (m, m), int(BOYUT * 0.47), (*MAVI, 255), -1, cv2.LINE_AA)
    ok = np.array([[int(BOYUT * 0.24), m],
                   [int(BOYUT * 0.50), int(BOYUT * 0.26)],
                   [int(BOYUT * 0.50), int(BOYUT * 0.41)],
                   [int(BOYUT * 0.76), int(BOYUT * 0.41)],
                   [int(BOYUT * 0.76), int(BOYUT * 0.59)],
                   [int(BOYUT * 0.50), int(BOYUT * 0.59)],
                   [int(BOYUT * 0.50), int(BOYUT * 0.74)]], np.int32)
    cv2.fillPoly(t, [ok], (*BEYAZ, 255), cv2.LINE_AA)
    return t


def saga_don():
    t = sola_don()
    return cv2.flip(t, 1)


def dur():
    """Kirmizi sekizgen + STOP. Referans: zaten calistigi bilinen tip."""
    t = _tuval((0, 0, 0, 0))
    m = BOYUT // 2
    r = int(BOYUT * 0.47)
    kose = np.array([[int(m + r * np.cos(np.pi / 8 + i * np.pi / 4)),
                      int(m + r * np.sin(np.pi / 8 + i * np.pi / 4))]
                     for i in range(8)], np.int32)
    cv2.fillPoly(t, [kose], (*KIRMIZI, 255), cv2.LINE_AA)
    (w, h), _ = cv2.getTextSize('STOP', cv2.FONT_HERSHEY_DUPLEX, 3.6, 18)
    cv2.putText(t, 'STOP', (m - w // 2, m + h // 2),
                cv2.FONT_HERSHEY_DUPLEX, 3.6, (*BEYAZ, 255), 18,
                cv2.LINE_AA)
    return t


# (dosya adi, uretec, YOLO'da beklenen sinif)
TABELALAR = [
    ('hiz_20', lambda: hiz_limiti(20), 'Speed Limit 20 KMPh'),
    ('hiz_30', lambda: hiz_limiti(30), 'Speed Limit 30 KMPh'),
    ('giris_yok', giris_yok, 'No Entry'),
    ('yol_ver', yol_ver, 'Give Way'),
    ('yaya_mavi', yaya_gecidi_mavi, 'Pedestrian Crossing'),
    ('yaya_ucgen', yaya_gecidi_ucgen, 'Pedestrian Crossing'),
    ('donel', donel_kavsak, 'Round-About'),
    ('sola_don', sola_don, 'Turn left ahead'),
    ('saga_don', saga_don, 'Turn right ahead'),
    ('dur', dur, 'Stop_Sign'),
]


def uret(cikti_dizin):
    os.makedirs(cikti_dizin, exist_ok=True)
    yollar = []
    for ad, fn, beklenen in TABELALAR:
        img = fn()
        yol = os.path.join(cikti_dizin, f'{ad}.png')
        cv2.imwrite(yol, img)
        yollar.append((ad, yol, beklenen))
    return yollar


def test_et(yollar, model_yolu, guven=0.25):
    """YOLO'yu uretilen dokular uzerinde dogrudan calistir.

    Tabela dokusu seffaf arka planli; YOLO'ya vermeden once gercekci
    bir sahneye benzetmek icin gri zemine yerlestiriliyor.
    """
    from ultralytics import YOLO
    model = YOLO(model_yolu)

    print(f'{"tabela":12} | {"beklenen sinif":22} | sonuc')
    print('-' * 70)
    basarili = 0
    for ad, yol, beklenen in yollar:
        rgba = cv2.imread(yol, cv2.IMREAD_UNCHANGED)
        # seffafligi gri zemine yedir, kenarlarda pay birak
        h, w = rgba.shape[:2]
        tuval = np.full((int(h * 1.6), int(w * 1.6), 3), 128, np.uint8)
        ox, oy = (tuval.shape[1] - w) // 2, (tuval.shape[0] - h) // 2
        alfa = rgba[:, :, 3:4].astype(float) / 255.0
        bolge = tuval[oy:oy + h, ox:ox + w]
        tuval[oy:oy + h, ox:ox + w] = (
            rgba[:, :, :3] * alfa + bolge * (1 - alfa)).astype(np.uint8)

        r = model(tuval, conf=guven, verbose=False)[0]
        bulunan = [(model.names[int(b.cls[0])], float(b.conf[0]))
                   for b in r.boxes]
        if not bulunan:
            print(f'{ad:12} | {beklenen:22} | --- tespit yok')
            continue
        bulunan.sort(key=lambda t: -t[1])
        dogru = any(s == beklenen for s, _ in bulunan)
        isaret = 'DOGRU' if dogru else 'yanlis'
        if dogru:
            basarili += 1
        print(f'{ad:12} | {beklenen:22} | {isaret}: '
              + ', '.join(f'{s}({c:.2f})' for s, c in bulunan[:3]))
    print('-' * 70)
    print(f'{basarili}/{len(yollar)} tabela dogru siniflandirildi '
          f'(guven esigi {guven})')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--cikti', default=None, help='doku cikti dizini')
    p.add_argument('--model', default=None, help='YOLO agirlik yolu')
    p.add_argument('--guven', type=float, default=0.25)
    p.add_argument('--sadece-uret', action='store_true')
    a = p.parse_args()

    kok = os.path.join(os.path.dirname(__file__), '..')
    cikti = a.cikti or os.path.join(kok, 'models', 'tabelalar', 'textures')
    model = a.model or os.path.join(kok, 'perception_pkg', 'weights',
                                    'best.pt')

    yollar = uret(cikti)
    print(f'{len(yollar)} doku uretildi -> {cikti}\n')
    if not a.sadece_uret:
        test_et(yollar, model, a.guven)


if __name__ == '__main__':
    main()
