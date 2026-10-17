#!/usr/bin/env python3
"""YOLOv8'i sentetik Gazebo verisiyle fine-tune eder.

Mevcut model (GTSRB benzeri gercek foto ile egitilmis) Gazebo'daki
tabelalari tanimiyordu (baseline mAP50 %2.25). Bu script:
  1. mevcut best.pt'yi baslangic alir,
  2. tools/veri_topla.py ile toplanan otomatik etiketli sentetik veriyle
     fine-tune eder,
  3. sonucu weights/best_finetuned.pt olarak kaydeder.

Tam pipeline:
    python3 tools/tabela_uret.py            # tabela dokulari
    python3 tools/tabela_model_uret.py      # Gazebo modelleri
    # gzserver bos.world + topla_kam spawn (bkz. veri_topla.py basligi)
    python3 tools/veri_topla.py --kare-sinif 70 --cikti /tmp/tabela_veri
    python3 tools/model_egit.py --veri /tmp/tabela_veri

Olculen sonuc (40 epoch, RTX 4050, ~3.5 dk):
    mAP50  : 0.0225 -> 0.482   (21x)
    recall : cogu sinif 0 -> 0.45-0.67
    Zayif  : hiz limiti 20/30 ayrimi (rakam ayirt etme; daha cok veri
             veya yakin/yuksek cozunurluk ornek gerekir).
"""

import argparse
import os


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--veri', default='/tmp/tabela_veri',
                   help='YOLO veri dizini (data.yaml icermeli)')
    p.add_argument('--baslangic', default=None,
                   help='baslangic agirligi (varsayilan: weights/best.pt)')
    p.add_argument('--epoch', type=int, default=40)
    p.add_argument('--batch', type=int, default=16)
    p.add_argument('--lr', type=float, default=0.001)
    p.add_argument('--cikti', default=None,
                   help='kaydedilecek model (varsayilan: '
                        'weights/best_finetuned.pt)')
    a = p.parse_args()

    from ultralytics import YOLO

    kok = os.path.join(os.path.dirname(__file__), '..')
    baslangic = a.baslangic or os.path.join(
        kok, 'perception_pkg', 'weights', 'best.pt')
    cikti = a.cikti or os.path.join(
        kok, 'perception_pkg', 'weights', 'best_finetuned.pt')
    data_yaml = os.path.join(a.veri, 'data.yaml')

    m = YOLO(baslangic)
    sonuc = m.train(
        data=data_yaml, epochs=a.epoch, imgsz=640, batch=a.batch,
        lr0=a.lr, patience=12, device=0,
        project='/tmp/egitim', name='tabela_ft', exist_ok=True,
        verbose=True, plots=True)

    egitilmis = os.path.join('/tmp/egitim', 'tabela_ft', 'weights', 'best.pt')
    import shutil
    shutil.copy(egitilmis, cikti)
    print(f'Fine-tune bitti -> {cikti}')
    if sonuc is not None:
        print('Sonuclar /tmp/egitim/tabela_ft/ altinda')


if __name__ == '__main__':
    main()
