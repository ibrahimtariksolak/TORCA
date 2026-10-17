#!/usr/bin/env python3
"""Uretilen tabela dokularini Gazebo modellerine cevirir.

Her tabela icin bir Gazebo model dizini olusturur:
    models/tabela_<ad>/
        model.sdf                       (dik duran dokulu levha + direk)
        model.config
        materials/scripts/<ad>.material (Ogre, alpha blend)
        materials/textures/<ad>.png     (doku, tabela_uret.py'den)

Boylece dokular simulasyonda GERCEK tabela gibi render edilir ve
YOLO fine-tune verisi dogrudan deployment domaininde toplanir.

Tabela dunya cercevesinde: levha XZ duzleminde (ince, Y yonunde),
yuzu +X'e bakar; dolayisiyla yaw ile dondurulup araca cevrilebilir.
Levha merkezi direk ustunde LEVHA_MERKEZ_Z yuksekliginde.
"""

import os
import shutil

# Fiziksel boyutlar (m) -- auto-label projeksiyonu bunlari kullanir
LEVHA_BOYUT = 0.90        # kare levha kenar uzunlugu
LEVHA_KALIN = 0.04
DIREK_YUKSEK = 2.0        # zeminden levha ALT kenarina
LEVHA_MERKEZ_Z = DIREK_YUKSEK + LEVHA_BOYUT / 2.0


def _material(ad):
    return f"""material tabela_{ad}/Diffuse
{{
  technique
  {{
    pass
    {{
      scene_blend alpha_blend
      depth_write on
      texture_unit
      {{
        texture {ad}.png
        filtering trilinear
      }}
    }}
  }}
}}
"""


def _model_sdf(ad):
    # Levha: ince kutu, kalinlik Y boyunca (araca yuz +X'e bakar).
    # Doku iki yuze de uygulanir; <visual> doku kaplamasi icin levha
    # yuzeyi XZ. Kutu boyutu (kalin, boyut, boyut) -> yuz +-X normalli.
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="tabela_{ad}">
    <static>true</static>
    <link name="link">
      <!-- direk -->
      <visual name="direk">
        <pose>0 0 {DIREK_YUKSEK/2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>{DIREK_YUKSEK:.3f}</length></cylinder></geometry>
        <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
      </visual>
      <!-- levha: yuz +-X normalli, doku kapli -->
      <visual name="levha">
        <pose>0 0 {LEVHA_MERKEZ_Z:.3f} 0 0 0</pose>
        <geometry>
          <box><size>{LEVHA_KALIN:.3f} {LEVHA_BOYUT:.3f} {LEVHA_BOYUT:.3f}</size></box>
        </geometry>
        <material>
          <script>
            <uri>model://tabela_{ad}/materials/scripts</uri>
            <uri>model://tabela_{ad}/materials/textures</uri>
            <name>tabela_{ad}/Diffuse</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _config(ad):
    return f"""<?xml version="1.0"?>
<model>
  <name>tabela_{ad}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Otomatik uretilmis dokulu trafik tabelasi: {ad}</description>
</model>
"""


def uret(doku_dizin, model_kok):
    import glob
    dokular = sorted(glob.glob(os.path.join(doku_dizin, '*.png')))
    uretilenler = []
    for yol in dokular:
        ad = os.path.splitext(os.path.basename(yol))[0]
        mdir = os.path.join(model_kok, f'tabela_{ad}')
        os.makedirs(os.path.join(mdir, 'materials', 'scripts'), exist_ok=True)
        os.makedirs(os.path.join(mdir, 'materials', 'textures'), exist_ok=True)
        shutil.copy(yol, os.path.join(mdir, 'materials', 'textures',
                                      f'{ad}.png'))
        with open(os.path.join(mdir, 'materials', 'scripts',
                               f'{ad}.material'), 'w') as f:
            f.write(_material(ad))
        with open(os.path.join(mdir, 'model.sdf'), 'w') as f:
            f.write(_model_sdf(ad))
        with open(os.path.join(mdir, 'model.config'), 'w') as f:
            f.write(_config(ad))
        uretilenler.append(ad)
    return uretilenler


def main():
    kok = os.path.join(os.path.dirname(__file__), '..')
    doku = os.path.join(kok, 'models', 'tabelalar', 'textures')
    # Modeller ~/.gazebo/models'e konur ki model:// ile bulunabilsin
    hedef = os.path.expanduser('~/.gazebo/models')
    adlar = uret(doku, hedef)
    print(f'{len(adlar)} tabela modeli uretildi -> {hedef}')
    print(f'levha {LEVHA_BOYUT} m, merkez z={LEVHA_MERKEZ_Z:.2f} m')
    for a in adlar:
        print(f'  tabela_{a}')


if __name__ == '__main__':
    main()
