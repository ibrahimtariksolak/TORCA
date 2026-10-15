"""IPM homografisinin dogrulugu icin birim testler.

Analitik gidis (zemin -> piksel) ve donus (piksel -> zemin) birbirinin
tersi olmak zorunda; kayan nokta hassasiyeti icinde kontrol ediliyor.
Ayrica ufuk ustu izdusumleri None dondurmeli, kus bakisi homografisi
geometrik olarak tutarli olmali.
"""
import math

import pytest

from perception_pkg.ipm import IPM


# urdf'teki gercek degerler
URDF = dict(genislik=640, yukseklik=480, yatay_fov=1.40,
            kamera_yuksek=1.30, kamera_ileri=1.95, kamera_egim=0.10)


@pytest.fixture
def ipm():
    return IPM(**URDF, x_min=5.0, x_max=25.0, y_yari=6.0)


def test_gidis_donus_birebir(ipm):
    """Zemin -> piksel -> zemin donusumu birebir olmali."""
    for X in (6, 10, 15, 20):
        for Y in (-5, -1.75, 0.0, 1.75, 5):
            p = ipm.zemin_to_piksel(X, Y)
            assert p is not None, f'({X},{Y}) kamera arkasinda cikti'
            g = ipm.piksel_to_zemin(*p)
            assert g is not None, f'({X},{Y}) ufuk ustu doner'
            assert math.hypot(g[0] - X, g[1] - Y) < 1e-9, \
                f'({X},{Y}) -> {p} -> {g} birebir degil'


def test_ufuk_ustu_none_doner(ipm):
    """Yatay ufuk cizgisinin ustundeki pikseller zemine izdusemez."""
    # 480 pikselde ufuk ~v=200 civari (kamera egimine gore)
    for v in (0, 50, 100, 150):
        assert ipm.piksel_to_zemin(320, v) is None, \
            f'v={v} ufuk ustu olmaliydi'


def test_kus_bakisi_boyutu(ipm):
    """Kus bakisi goruntunun boyutu x_max-x_min ve 2*y_yari ile uyumlu."""
    beklenen_g = int(round(2 * 6.0 / ipm.cozunurluk))
    beklenen_y = int(round((25.0 - 5.0) / ipm.cozunurluk))
    assert ipm.kb_g == beklenen_g
    assert ipm.kb_y == beklenen_y


def test_kus_bakisi_satir_sutun_cevrimi(ipm):
    """kb_satir_to_x ve kb_sutun_to_y tutarli olmali."""
    # ust satir en uzak noktaya karsilik gelmeli
    assert ipm.kb_satir_to_x(0) == pytest.approx(ipm.x_max, abs=1e-9)
    assert ipm.kb_satir_to_x(ipm.kb_y - 1) == pytest.approx(
        ipm.x_min + ipm.cozunurluk, abs=0.1)
    # ortanin ortasi Y=0
    assert ipm.kb_sutun_to_y(ipm.kb_g / 2) == pytest.approx(0.0, abs=0.1)


def test_ileri_yon_sagda_dusuk_piksel(ipm):
    """Araç cercevesinde +Y (sol) kucuk piksel sutununa dusmeli."""
    sag = ipm.zemin_to_piksel(10, -1.75)
    sol = ipm.zemin_to_piksel(10, +1.75)
    assert sol[0] < sag[0], 'sol dunya noktasi sag pikselden kucuk olmali'


def test_yakin_mesafe_asagi_dusmeli(ipm):
    """Araça yakin yer noktasi goruntunun alt kismina dusmeli."""
    yakin = ipm.zemin_to_piksel(6, 0)
    uzak = ipm.zemin_to_piksel(20, 0)
    assert yakin[1] > uzak[1], \
        'yakin nokta buyuk v degerine (asagi) dusmeli'
