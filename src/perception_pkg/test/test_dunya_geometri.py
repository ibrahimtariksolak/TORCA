"""Dunya ureteci geometrisinin birim testleri.

generate_otoban.py'nin orta hat ve ofset hat fonksiyonlari matematiksel
olarak tutarli olmali: oval kapali, ofset hatlar orta hattan tam |ofset|
uzakta.
"""
import math
import os
import sys

import pytest

# generate_otoban.py bir ROS modulu degil, worlds/ altinda bagimsiz script
WORLDS = os.path.join(os.path.dirname(__file__), '..', 'worlds')
sys.path.insert(0, WORLDS)

import generate_otoban as go  # noqa: E402


DUZ = 60.0
R = 25.0


def test_orta_hat_kapali():
    """Orta hat bir oval, basi ve sonu ayni noktaya denk gelmeli."""
    L = go.merkez_hat_uzunluk(DUZ, R)
    bas = go.merkez_hat_nokta(0.0, DUZ, R)
    son = go.merkez_hat_nokta(L, DUZ, R)
    assert math.hypot(bas[0] - son[0], bas[1] - son[1]) < 1e-10


def test_ofset_hat_kapali():
    """Her ofset hat da kapali olmali."""
    for ofset in (-3.5, 0.0, 3.5):
        L = go.ofset_hat_uzunluk(ofset, DUZ, R)
        bas = go.ofset_hat_nokta(0.0, ofset, DUZ, R)
        son = go.ofset_hat_nokta(L, ofset, DUZ, R)
        assert math.hypot(bas[0] - son[0], bas[1] - son[1]) < 1e-10, \
            f'ofset={ofset} hat kapali degil'


def test_ofset_hat_uzunluklari_dogru():
    """Duz bolumler degismez; viraj kismi (R-ofset) yarıcapli."""
    for ofset in (-3.5, 0.0, 3.5):
        L = go.ofset_hat_uzunluk(ofset, DUZ, R)
        beklenen = 2 * DUZ + 2 * math.pi * (R - ofset)
        assert L == pytest.approx(beklenen, abs=1e-9)


def test_ofset_hat_orta_hattan_mesafe():
    """Ofset hat uzerindeki her nokta orta hattan |ofset| uzakta olmali."""
    for ofset in (-3.5, 3.5):
        L = go.ofset_hat_uzunluk(ofset, DUZ, R)
        Lc = go.merkez_hat_uzunluk(DUZ, R)
        kotu = 0.0
        # orta hattan en yakin mesafeyi kaba tarama ile hesapla
        for i in range(50):
            t = L * i / 50
            px, py, _ = go.ofset_hat_nokta(t, ofset, DUZ, R)
            en_yakin = 1e9
            for j in range(1000):
                cx, cy, _ = go.merkez_hat_nokta(Lc * j / 1000, DUZ, R)
                en_yakin = min(en_yakin, math.hypot(px - cx, py - cy))
            kotu = max(kotu, abs(en_yakin - abs(ofset)))
        # tarama cozunurlugu kaynakli hata < 10 cm
        assert kotu < 0.1, f'ofset={ofset} mesafe sapmasi {kotu:.3f}'


def test_spawn_pozu_sag_seritte():
    """Spawn pozu orta hattan tam serit/2 kadar sagda olmali."""
    serit = 3.5
    x, y, _ = go.spawn_pozu(DUZ, R, serit)
    # alt duz bolumde orta hat y=-R, sag serit merkezi y=-R-serit/2
    beklenen_y = -R - serit / 2.0
    assert y == pytest.approx(beklenen_y, abs=1e-6)
    assert -DUZ / 2 < x < DUZ / 2, 'spawn alt duz bolumun icinde olmali'


def test_dunya_sdf_gecerli(tmp_path):
    """Uretilen SDF xml.etree ile ayristirilabilmeli ve state blogu
    icermemeli (GUI "Save World" bozulma onlemi)."""
    import xml.etree.ElementTree as ET
    sdf = go.dunya_uret()
    f = tmp_path / 'test.world'
    f.write_text(sdf)
    kok = ET.parse(f).getroot()
    dunya = kok.find('world')
    assert dunya is not None
    assert len(dunya.findall('state')) == 0, 'state blogu olmamali'
    # beklenen modeller var mi
    adlar = [m.get('name') for m in dunya.findall('model')]
    for gereken in ('zemin', 'yol_yuzeyi', 'dis_kenar_cizgi',
                    'ic_kenar_cizgi', 'orta_cizgi'):
        assert gereken in adlar, f'{gereken} modeli uretilmedi'
