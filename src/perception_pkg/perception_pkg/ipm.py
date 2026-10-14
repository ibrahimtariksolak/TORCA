#!/usr/bin/env python3
"""Ters perspektif donusumu (IPM / kus bakisi).

Homografi 4 nokta elle secilerek degil, kameranin bilinen ic ve dis
parametrelerinden ANALITIK turetilir. Boylece kamera yuksekligi veya
egimi urdf'te degistiginde burasi kendiliginde dogru kalir.

Cerceveler
----------
Araç  : X ileri, Y sola, Z yukari
Optik : x saga, y asagi, z optik eksen boyunca ileri

Egimi theta kadar asagi bakan kamera icin optik taban vektorlerinin
araç cercevesindeki karsiliklari:

    z_o -> ( cos t,  0, -sin t)
    x_o -> ( 0,     -1,  0    )
    y_o -> (-sin t,  0, -cos t)

Taban ortonormal oldugundan izdusum basitce ic carpimla bulunur.
"""

import math

import cv2
import numpy as np


class IPM:
    def __init__(self, genislik, yukseklik, yatay_fov,
                 kamera_yuksek, kamera_ileri, kamera_egim,
                 x_min=4.0, x_max=30.0, y_yari=7.0, cozunurluk=0.05):
        """
        genislik/yukseklik : goruntu boyutu (piksel)
        yatay_fov          : yatay gorus acisi (rad)
        kamera_yuksek      : kameranin yerden yuksekligi (m)
        kamera_ileri       : kameranin arac merkezinden ileriligi (m)
        kamera_egim        : asagi bakis acisi (rad)
        x_min/x_max        : kus bakisi goruntunun ileri menzili (m)
        y_yari             : yanal yari genislik (m)
        cozunurluk         : kus bakisi goruntude metre/piksel
        """
        self.G, self.Y = genislik, yukseklik
        self.h = kamera_yuksek
        self.x_cam = kamera_ileri
        self.t = kamera_egim

        # Gazebo kamerasi kare piksel uretir: fy = fx
        self.fx = (genislik / 2.0) / math.tan(yatay_fov / 2.0)
        self.fy = self.fx
        self.cx = genislik / 2.0
        self.cy = yukseklik / 2.0

        self.x_min, self.x_max = x_min, x_max
        self.y_yari = y_yari
        self.cozunurluk = cozunurluk

        # Kus bakisi goruntu boyutu
        self.kb_g = int(round(2 * y_yari / cozunurluk))
        self.kb_y = int(round((x_max - x_min) / cozunurluk))

        self.H = self._homografi()

    # ---- temel izdusum ----
    def zemin_to_piksel(self, X, Y):
        """Zemin noktasi (X ileri, Y sola) -> goruntu pikseli (u, v)."""
        dx = X - self.x_cam
        ct, st = math.cos(self.t), math.sin(self.t)
        a = -Y
        b = -dx * st + self.h * ct
        c = dx * ct + self.h * st
        if c <= 1e-6:                       # kameranin arkasi
            return None
        return (self.cx + self.fx * a / c,
                self.cy + self.fy * b / c)

    def piksel_to_zemin(self, u, v):
        """Goruntu pikseli -> zemin noktasi. Ufuk ustu icin None."""
        a = (u - self.cx) / self.fx
        b = (v - self.cy) / self.fy
        ct, st = math.cos(self.t), math.sin(self.t)
        # isin yonu araç cerceveside
        Dx = -b * st + ct
        Dy = -a
        Dz = -b * ct - st
        if Dz >= -1e-9:                     # ufka paralel veya yukari
            return None
        k = -self.h / Dz
        return (self.x_cam + k * Dx, k * Dy)

    # ---- kus bakisi ----
    def _kb_to_zemin(self, px, py):
        """Kus bakisi pikseli -> zemin. Ust satir en uzak, sol sutun +Y."""
        X = self.x_max - py * self.cozunurluk
        Y = self.y_yari - px * self.cozunurluk
        return X, Y

    def _homografi(self):
        """Goruntu -> kus bakisi homografisi.

        Kus bakisi goruntunun dort kosesine karsilik gelen zemin
        noktalarini kamera goruntusune izdusurup eslesmeyi kuruyoruz.
        """
        kb_kose = [(0, 0), (self.kb_g - 1, 0),
                   (self.kb_g - 1, self.kb_y - 1), (0, self.kb_y - 1)]
        kaynak, hedef = [], []
        for px, py in kb_kose:
            X, Y = self._kb_to_zemin(px, py)
            p = self.zemin_to_piksel(X, Y)
            if p is None:
                raise ValueError(
                    f'kus bakisi kosesi ({X:.1f}, {Y:.1f}) kamera arkasinda; '
                    'x_min/x_max veya y_yari degerlerini kucult')
            kaynak.append(p)
            hedef.append((float(px), float(py)))
        return cv2.getPerspectiveTransform(
            np.array(kaynak, dtype=np.float32),
            np.array(hedef, dtype=np.float32))

    def cevir(self, goruntu):
        """Goruntuyu kus bakisina cevirir."""
        return cv2.warpPerspective(goruntu, self.H, (self.kb_g, self.kb_y),
                                   flags=cv2.INTER_NEAREST)

    def kb_satir_to_x(self, py):
        """Kus bakisi satiri -> ileri mesafe (m)."""
        return self.x_max - py * self.cozunurluk

    def kb_sutun_to_y(self, px):
        """Kus bakisi sutunu -> yanal mesafe (m, sola pozitif)."""
        return self.y_yari - px * self.cozunurluk

    def y_to_kb_sutun(self, Y):
        return (self.y_yari - Y) / self.cozunurluk
