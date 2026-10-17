# ROS 2 Otonom Araç Algılama ve Kontrol

Gazebo'da simüle edilen otonom bir araç için ROS 2 (Humble) algılama + kontrol yığını:
Ackermann direksiyonlu gerçek ölçekli araç, kamera tabanlı şerit takibi (HSV + kuş
bakışı + saf takip), YOLOv8 ile trafik tabelası tanıma ve LiDAR tabanlı acil fren.

---

## Hızlı başlangıç

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_pkg
source install/setup.bash
ros2 launch perception_pkg otonom_surus.launch.py
```

Araç dünyada doğar, şerit takip ederek oval pisti süresiz döner, dur tabelasını
görünce durur, önüne engel çıkarsa güvenle frenler.

### Launch argümanları

| argüman | varsayılan | ne işe yarar |
|---|---|---|
| `world` | `otoban.world` | Yüklenecek Gazebo dünyası |
| `gui` | `true` | gzclient (Gazebo arayüzü) açılsın mı |
| `rviz` | `true` | RViz açılsın mı |
| `algilama` | `true` | sign_detector + serit_takip çalışsın mı |
| `guvenlik` | `true` | LiDAR tabanlı acil fren katmanı aktif mi |
| `use_sim_time` | `true` | Düğümler Gazebo saatini kullansın |

Örnekler:

```bash
# Başsız (gui/rviz'siz), araç dinamiği testi için algılamayı da kapat
ros2 launch perception_pkg otonom_surus.launch.py gui:=false rviz:=false algilama:=false

# Engel senaryosunu yükle (önce dünyayı üret)
python3 src/perception_pkg/worlds/generate_otoban.py --engel-s 45 -o engel.world
ros2 launch perception_pkg otonom_surus.launch.py world:=$PWD/engel.world

# JÜRİ İÇİN TEK KOMUT: tüm yetenekleri birden gösteren showcase
python3 src/perception_pkg/worlds/generate_otoban.py --showcase -o showcase.world
ros2 launch perception_pkg otonom_surus.launch.py trafik:=3 world:=$PWD/showcase.world
```

Showcase içinde: fine-tune ile tanınan 5 tabela çeşidi (hız limiti, yaya,
dönel kavşak, dur, sağa dön), zebra geçidi, duran bir engel ve 3 renkli
trafik aracı birden. Tek turda tüm yeteneklerin ard arda çalıştığı görülür.

---

## Mimari

```
                    ┌─────────────────────┐
   /arac/kamera/    │                     │  /detected_sign
   image_raw ─────▶ │   sign_detector     │ ───────────┐
                    │   (YOLOv8, 6 Hz)    │            │
                    └─────────────────────┘            │
                             │ /sign_detector/annotated │
                             ▼                          ▼
                    ┌─────────────────────┐    ┌─────────────────────┐
   /arac/kamera/    │   serit_takip       │    │ tabela reaksiyonu   │
   image_raw ─────▶ │   HSV + IPM +       │ ◀──┤ (hız limiti, DUR,   │
   /arac/odom  ───▶ │   saf takip         │    │  yol ver, yaya...)  │
                    └─────────────────────┘    └─────────────────────┘
                             │ /arac/cmd_vel_cmd
                             ▼
                    ┌─────────────────────┐
   /arac/scan  ───▶ │   guvenlik          │ ──▶ /arac/cmd_vel
                    │   (LiDAR acil fren) │
                    └─────────────────────┘
                             │
                             ▼
                      ackermann_drive eklentisi
```

`serit_takip` doğrudan `cmd_vel`'e yazmıyor; `cmd_vel_cmd` üzerinden `guvenlik`
düğümüne gidiyor. Güvenlik LiDAR'da engel görürse hız komutunu kırpıyor veya
sıfırlıyor. `guvenlik:=false` ile launch bu düğümü atlayıp doğrudan bağlantı kuruyor.

---

## Teknik yığın

| Bileşen | Teknoloji |
|---|---|
| Orta katman | ROS 2 Humble (rclpy) |
| Simülasyon | Gazebo Classic 11, ackermann_drive eklentisi |
| Araç | 4.0 × 1.8 m, 2.5 m dingil, Ackermann direksiyon, 1200 kg |
| Sensörler | Kamera (640×480, 80° FOV, 30 Hz) · 2D LiDAR (±90°, 60 m, 15 Hz) · IMU (100 Hz) |
| Şerit takibi | HSV maske + IPM homografi + saf takip (pure pursuit) |
| Tabela tanıma | YOLOv8 (ultralytics), 6 Hz throttle, bbox + sınıf süzgeci |
| Güvenlik | LiDAR koridor taraması + fren ivmesiyle tutarlı yavaşlama |
| Yapı | ament_python, colcon |

---

## Ölçülen davranış

Rakamlar `tools/` altındaki ölçüm araçlarıyla toplandı. Tahmin yok.

**Oval pistte 150 s'lik şerit takibi:**

```
kat edilen yol  : 590 m (2.13 tur)
ort hiz         : 3.96 m/s
şerit sapması   : ort 0.14 m, maks 0.85 m
   düz bölümde  : 0.27 m
   virajda      : 0.07 m
```

**Engel senaryosu** (araç yolunda duran 1.2 m genişliğinde kutuya yaklaşıyor):

```
güvenlik AÇIK:  araç engelin 2.0 m öncesinde yavaşlayarak durdu
güvenlik KAPALI: kontrolcü 6.88 m/s komut vermeye devam ediyor,
                 araç engele dayalı duruyor (algılama yok)
```

---

## Proje yapısı

```
src/perception_pkg/
├── perception_pkg/
│   ├── serit_takip.py     # HSV + IPM + saf takip kontrolcüsü
│   ├── sign_detector.py   # YOLOv8 tabela algılama
│   ├── guvenlik.py        # LiDAR tabanlı acil fren
│   ├── ipm.py             # analitik ters perspektif dönüşümü
│   └── weights/best.pt    # YOLOv8 ağırlıkları
├── launch/otonom_surus.launch.py
├── urdf/otonom_arac.urdf.xacro   # Ackermann araç, kamera, IMU, LiDAR
├── worlds/
│   ├── otoban.world              # OTOMATIK ÜRETILIR, elle düzenleme
│   └── generate_otoban.py        # oval pisti parametrik üreten script
├── models/                       # Gazebo kamera modelleri
└── tools/                        # ölçüm araçları (aşağıda)
```

**Dünya elle düzenlenmez.** Gazebo GUI'den "Save World" bir `<state>` bloğu yazıp
model pozlarını sessizce eziyor. Pisti değiştirmek için `generate_otoban.py`'ı
düzenleyip yeniden çalıştırın.

---

## Ölçüm araçları (`tools/`)

| araç | ne ölçer |
|---|---|
| `tur_testi.py` | kaç tur atıldı, şerit sapması (düz/viraj ayrı), ortalama hız |
| `sapma_tani.py` | sapmanın kaynağı: algıda mı yoksa kontrolde mi |
| `tabela_kalibre.py` | tabela mesafesi ↔ bbox yüksekliği eşlemesi |
| `engel_testi.py` | engele en yakın mesafe + komut hızı (güvenlik var/yok karşılaştırması) |

Bu araçlar jüriye `değerlendirme kriterim ölçülebilir` demek için var. Örnek:

```bash
python3 src/perception_pkg/tools/tur_testi.py 150
```

---

## Önkoşullar

- ROS 2 Humble (`gazebo_ros`, `cv_bridge`, `rviz2`, `xacro`)
- Python 3.10+
- `ultralytics`, `opencv-python`, `numpy`
- `~/.gazebo/models/` altında `asphalt_plane`, `stop_sign`, `speed_limit_sign`
  (standart Gazebo model deposundan iner)

---

## Lisans

Apache-2.0. Bakımcı: Ibrahim Tarik Solak <ibrahimtariksolak.is@gmail.com>
