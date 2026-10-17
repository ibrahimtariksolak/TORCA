#!/usr/bin/env python3
"""Sentetik YOLO egitim verisi toplar (otomatik etiketli).

Yontem: kamera sabit (araç kamerasiyla ayni ic/dis parametreler),
tabelalar cesitli mesafe/yanal konum/aci ile ONUNE spawn edilir. Her
kare icin tabelanin 3B levha kutusu projeksiyonla goruntuye izdusurulup
YOLO formatinda bounding box etiketi OTOMATIK uretilir (elle etiket yok).

Etiket dogrulugu tools ile gorsel olarak dogrulandi (bkz. oturum).

Kullanim (gzserver bos.world ile calismali, kamera 'topla_kam' origin'de):
    python3 tools/veri_topla.py --kare-sinif 60 --cikti /tmp/tabela_veri
"""

import argparse
import math
import os
import random
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'perception_pkg'))
from ipm import IPM  # noqa: E402

# Araç kamerasiyla ayni (config/params.yaml ile tutarli)
FOV = 1.40
GEN, YUK = 640, 480
KAM_Z = 1.30
KAM_PITCH = 0.10
LEVHA = 0.90          # tabela_model_uret.py ile ayni
LEVHA_MERKEZ_Z = 2.45

# tabela modeli -> YOLO sinif indeksi (orijinal 29-sinif modeli)
TABELA_SINIF = {
    'hiz_20': 20, 'hiz_30': 21, 'giris_yok': 14, 'yol_ver': 8,
    'yaya_mavi': 17, 'yaya_ucgen': 17, 'yaya_eski': 17,
    'donel': 18, 'sola_don': 26, 'saga_don': 27, 'dur': 22,
}


def levha_bbox(ipm, X, Y):
    """Kamera cercevesinde (X ileri, Y sola) tabela levhasinin 2B kutusu.

    Levha merkezi z=LEVHA_MERKEZ_Z, kenar LEVHA. 8 kose izdusurulup
    eksen hizali kutu alinir.
    """
    us, vs = [], []
    for dy in (-LEVHA / 2, LEVHA / 2):
        for dz in (-LEVHA / 2, LEVHA / 2):
            for dx in (-0.03, 0.03):
                p = ipm.nokta_to_piksel(X + dx, Y + dy, LEVHA_MERKEZ_Z + dz)
                if p is None:
                    return None
                us.append(p[0])
                vs.append(p[1])
    x1, y1, x2, y2 = min(us), min(vs), max(us), max(vs)
    # goruntu disina tasan kismi kirp
    x1, x2 = max(0, x1), min(GEN, x2)
    y1, y2 = max(0, y1), min(YUK, y2)
    if x2 - x1 < 8 or y2 - y1 < 8:
        return None              # cok kucuk / gorunmuyor
    return x1, y1, x2, y2


class Toplayici(Node):
    def __init__(self):
        super().__init__('veri_topla')
        self.br = CvBridge()
        self.son_kare = None
        self.create_subscription(Image, '/topla_kam/image_raw',
                                 self._kare, 10)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.sil = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn.wait_for_service(timeout_sec=10)
        self.sil.wait_for_service(timeout_sec=10)
        self.ipm = IPM(GEN, YUK, FOV, KAM_Z, 0.0, KAM_PITCH)

    def _kare(self, msg):
        self.son_kare = msg

    def _yeni_kare(self, zaman_asimi=3.0):
        self.son_kare = None
        t0 = time.time()
        while time.time() - t0 < zaman_asimi:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.son_kare is not None:
                return self.br.imgmsg_to_cv2(self.son_kare, 'bgr8')
        return None

    def tabela_spawn(self, model_ad, d, lat, yaw):
        sdf = open(os.path.expanduser(
            f'~/.gazebo/models/tabela_{model_ad}/model.sdf')).read()
        req = SpawnEntity.Request()
        req.name = 'hedef_tabela'
        req.xml = sdf
        req.initial_pose.position.x = float(d)
        req.initial_pose.position.y = float(lat)
        req.initial_pose.orientation.z = math.sin(yaw / 2)
        req.initial_pose.orientation.w = math.cos(yaw / 2)
        fut = self.spawn.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)

    def tabela_sil(self):
        req = DeleteEntity.Request()
        req.name = 'hedef_tabela'
        fut = self.sil.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--kare-sinif', type=int, default=60)
    p.add_argument('--cikti', default='/tmp/tabela_veri')
    p.add_argument('--val-oran', type=float, default=0.15)
    p.add_argument('--seed', type=int, default=0)
    a = p.parse_args()
    random.seed(a.seed)

    for alt in ('images/train', 'images/val', 'labels/train', 'labels/val'):
        os.makedirs(os.path.join(a.cikti, alt), exist_ok=True)

    rclpy.init()
    n = Toplayici()
    sayac = 0
    atlanan = 0
    for model_ad, sinif in TABELA_SINIF.items():
        for i in range(a.kare_sinif):
            # cesitlilik: mesafe, yanal konum, yaw jitter
            d = random.uniform(4.5, 22.0)
            # yanal: uzaktayken genis, yakinda dar (cerceve icinde kalsin)
            lat_maks = min(3.0, 0.35 * d)
            lat = random.uniform(-lat_maks, lat_maks)
            yaw = math.pi + random.uniform(-0.35, 0.35)

            n.tabela_spawn(model_ad, d, lat, yaw)
            time.sleep(0.12)
            kare = n._yeni_kare()
            # etiket: kamera origin yaw0, tabela rel (X=d, Y=lat)
            bbox = levha_bbox(n.ipm, d, lat)
            n.tabela_sil()
            time.sleep(0.04)

            if kare is None or bbox is None:
                atlanan += 1
                continue

            val = random.random() < a.val_oran
            alt = 'val' if val else 'train'
            ad = f'{model_ad}_{i:03d}'
            cv2.imwrite(os.path.join(a.cikti, 'images', alt, f'{ad}.jpg'),
                        kare)
            x1, y1, x2, y2 = bbox
            cx = (x1 + x2) / 2 / GEN
            cy = (y1 + y2) / 2 / YUK
            bw = (x2 - x1) / GEN
            bh = (y2 - y1) / YUK
            with open(os.path.join(a.cikti, 'labels', alt, f'{ad}.txt'),
                      'w') as f:
                f.write(f'{sinif} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n')
            sayac += 1
            if sayac % 50 == 0:
                print(f'  {sayac} kare toplandi...')

    print(f'TOPLAM {sayac} kare (atlanan {atlanan}) -> {a.cikti}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
