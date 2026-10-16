#!/usr/bin/env python3
"""Gercek trafik tabelasi gorsellerini indirir ve YOLO testinden gecirir.

NEDEN: Elle cizilen tabelalar YOLO tarafindan taninmadi (48 varyant
denendi, hiz limiti ve yaya gecidi icin hepsi basarisiz). Model
GTSRB benzeri GERCEK tabela goruntuleriyle egitilmis, dolayisiyla
kaynak gorsellerin de o dagilima yakin olmasi gerekiyor.

KAYNAK: Wikimedia Commons uzerindeki Alman StVO tabelalari. Alman
hukukunda trafik tabelalari "amtliches Werk" (resmi eser) sayilir ve
kamu malidir; Commons bunlari serbest lisansla barindirir.

Kullanim:
    python3 tools/tabela_indir.py            # indir + test
    python3 tools/tabela_indir.py --listele  # sadece ne indirilecegini goster
"""

import argparse
import os
import sys
import urllib.parse
import urllib.request

# (yerel ad, Commons dosya adi, YOLO'da beklenen sinif)
TABELALAR = [
    ('hiz_20', 'Zeichen 274-20 - Zulässige Höchstgeschwindigkeit, StVO 2017.svg',
     'Speed Limit 20 KMPh'),
    ('hiz_30', 'Zeichen 274-30 - Zulässige Höchstgeschwindigkeit, StVO 2017.svg',
     'Speed Limit 30 KMPh'),
    ('giris_yok', 'Zeichen 267 - Verbot der Einfahrt, StVO 1970.svg',
     'No Entry'),
    # NOT: Commons dosya adlari noktalama acisindan hassas; asagidaki
    # adlar Commons arama API'si ile dogrulandi (virgul/unlem yerleri
    # tahmin edilirse 404 doner).
    ('yol_ver', 'Zeichen 205 - Vorfahrt gewähren! StVO 1970.svg',
     'Give Way'),
    ('yaya_mavi', 'Zeichen 350-10 - Fußgängerüberweg (rechts), StVO 1992.svg',
     'Pedestrian Crossing'),
    ('yaya_eski', 'Zeichen 350 - Fußgängerüberweg, StVO 1970.svg',
     'Pedestrian Crossing'),
    ('donel', 'Zeichen 215 - Kreisverkehr, StVO 2000.svg',
     'Round-About'),
    ('sola_don', 'Zeichen 209-10 - Vorgeschriebene Fahrtrichtung, links, StVO 1992.svg',
     'Turn left ahead'),
    ('saga_don', 'Zeichen 209-20 - Vorgeschriebene Fahrtrichtung, rechts, StVO 1992.svg',
     'Turn right ahead'),
    ('dur', 'Zeichen 206 - Halt! Vorfahrt gewähren! StVO 1970.svg',
     'Stop_Sign'),
]

GENISLIK = 512


def url(commons_adi):
    """Special:FilePath verilen genislikte PNG'ye render edip yonlendirir."""
    return ('https://commons.wikimedia.org/wiki/Special:FilePath/'
            + urllib.parse.quote(commons_adi.replace(' ', '_'))
            + f'?width={GENISLIK}')


def indir(cikti_dizin):
    os.makedirs(cikti_dizin, exist_ok=True)
    sonuc = []
    for ad, commons, beklenen in TABELALAR:
        hedef = os.path.join(cikti_dizin, f'{ad}.png')
        try:
            istek = urllib.request.Request(
                url(commons),
                headers={'User-Agent': 'perception_pkg/1.0 (ROS2 egitim projesi)'})
            with urllib.request.urlopen(istek, timeout=30) as r:
                veri = r.read()
            with open(hedef, 'wb') as f:
                f.write(veri)
            print(f'  OK   {ad:12} {len(veri)/1024:6.1f} KB  <- {commons[:48]}')
            sonuc.append((ad, hedef, beklenen))
        except Exception as e:
            print(f'  HATA {ad:12} {type(e).__name__}: {e}')
    return sonuc


def test_et(yollar, model_yolu, guven=0.25):
    """Indirilen gorselleri dogrudan YOLO'ya ver."""
    import cv2
    import numpy as np
    from ultralytics import YOLO
    model = YOLO(model_yolu)

    print()
    print(f'{"tabela":12} | {"beklenen":22} | sonuc')
    print('-' * 74)
    basarili = []
    for ad, yol, beklenen in yollar:
        rgba = cv2.imread(yol, cv2.IMREAD_UNCHANGED)
        if rgba is None:
            print(f'{ad:12} | {beklenen:22} | goruntu okunamadi')
            continue
        if rgba.shape[2] == 4:
            h, w = rgba.shape[:2]
            tuval = np.full((int(h * 1.5), int(w * 1.5), 3), 128, np.uint8)
            ox, oy = (tuval.shape[1] - w) // 2, (tuval.shape[0] - h) // 2
            a = rgba[:, :, 3:4].astype(float) / 255.0
            b = tuval[oy:oy + h, ox:ox + w]
            tuval[oy:oy + h, ox:ox + w] = (
                rgba[:, :, :3] * a + b * (1 - a)).astype(np.uint8)
        else:
            tuval = rgba

        r = model(tuval, conf=guven, verbose=False)[0]
        bulunan = sorted(((model.names[int(b.cls[0])], float(b.conf[0]))
                          for b in r.boxes), key=lambda t: -t[1])
        if not bulunan:
            print(f'{ad:12} | {beklenen:22} | --- tespit yok')
            continue
        dogru = any(s == beklenen for s, _ in bulunan)
        if dogru:
            basarili.append(ad)
        print(f'{ad:12} | {beklenen:22} | '
              f'{"DOGRU" if dogru else "yanlis"}: '
              + ', '.join(f'{s}({c:.2f})' for s, c in bulunan[:3]))
    print('-' * 74)
    print(f'{len(basarili)}/{len(yollar)} dogru: {", ".join(basarili)}')
    return basarili


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cikti', default=None)
    p.add_argument('--model', default=None)
    p.add_argument('--guven', type=float, default=0.25)
    p.add_argument('--listele', action='store_true')
    a = p.parse_args()

    kok = os.path.join(os.path.dirname(__file__), '..')
    cikti = a.cikti or os.path.join(kok, 'models', 'tabelalar', 'textures')
    model = a.model or os.path.join(kok, 'perception_pkg', 'weights', 'best.pt')

    if a.listele:
        print(f'Indirilecek {len(TABELALAR)} dosya ({GENISLIK}px PNG):')
        for ad, commons, beklenen in TABELALAR:
            print(f'  {ad:12} <- {commons}')
            print(f'               {url(commons)}')
        return

    print(f'{len(TABELALAR)} tabela indiriliyor -> {cikti}')
    yollar = indir(cikti)
    if not yollar:
        print('hicbiri indirilemedi')
        sys.exit(1)
    test_et(yollar, model, a.guven)


if __name__ == '__main__':
    main()
