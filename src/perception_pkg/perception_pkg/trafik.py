#!/usr/bin/env python3
"""Trafik simulasyonu: pistte kendi seridinde giden diger araclar.

Araclar pistin BILINEN orta hatti uzerinde script'le surulur:
her aracin bir s0 baslangic ofseti ve ortak bir hizi vardir, konum
s(t) = s0 + v*t olarak hesaplanip /gazebo/set_entity_state ile
yazilir. Esit aralikli ve ayni hizda olduklari icin birbirlerine
carpmalari MATEMATIKSEL OLARAK imkansiz; fizik motoruna is dusmez,
cok sayida araç eklenebilir.

Araclar statik olmayan gercek modellerdir, yani carpisma govdeleri
vardir ve bizim aracin LiDAR'i onlari dogal olarak gorur. Guvenlik
katmani onlerine gelindiginde durmayi kendisi halleder.

Not: /gazebo/set_entity_state servisi gazebo_ros_state eklentisinden
gelir ve o bir WorldPlugin'dir; gzserver'a -s ile verilirse sessizce
dusup servis hic olusmaz. Eklenti dunya dosyasinin icinde tanimli
olmak zorunda (bkz. worlds/generate_otoban.py).
"""

import math
import os
import sys

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node


def _pist_modulu():
    """Pist geometrisini saglayan generate_otoban modulunu yukler.

    Modul bir ROS paketi degil, worlds/ altinda bagimsiz bir script;
    setup.py onu share/perception_pkg/worlds/ altina kuruyor.
    """
    from ament_index_python.packages import get_package_share_directory
    worlds = os.path.join(get_package_share_directory('perception_pkg'),
                          'worlds')
    if worlds not in sys.path:
        sys.path.insert(0, worlds)
    import generate_otoban
    return generate_otoban


class Trafik(Node):

    def __init__(self):
        super().__init__('trafik')
        self.go = _pist_modulu()

        self.declare_parameter('arac_sayisi', 4)
        self.declare_parameter('hiz', 2.5)             # m/s
        self.declare_parameter('serit_ofset', -1.75)   # sag seridin ortasi
        self.declare_parameter('guncelleme_hz', 30.0)
        self.declare_parameter('baslangic_s', 60.0)    # bizim araca mesafe
        self.declare_parameter('arac_adlari', [''])

        g = self.get_parameter
        self.n_arac = g('arac_sayisi').value
        self.hiz = g('hiz').value
        self.ofset = g('serit_ofset').value
        self.bas_s = g('baslangic_s').value

        adlar = [a for a in g('arac_adlari').value if a]
        self.adlar = adlar or [f'trafik_{i+1}' for i in range(self.n_arac)]
        self.n_arac = len(self.adlar)

        self.duz = self.go.DUZ_UZUNLUK
        self.yaricap = self.go.VIRAJ_YARICAP
        self.hat_boy = self.go.ofset_hat_uzunluk(
            self.ofset, self.duz, self.yaricap)

        # Esit aralikli yerlesim -> aralarindaki mesafe sabit kalir
        self.aralik = self.hat_boy / self.n_arac

        self.cli = self.create_client(SetEntityState,
                                      '/gazebo/set_entity_state')
        if not self.cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                '/gazebo/set_entity_state yok. gazebo_ros_state eklentisi '
                'dunya dosyasinda tanimli mi? (-s ile yuklenmez)')
            raise RuntimeError('set_entity_state servisi bulunamadi')

        self.t0 = self._simdi()
        self.create_timer(1.0 / g('guncelleme_hz').value, self.tik)
        self.get_logger().info(
            f'Trafik basladi: {self.n_arac} araç, {self.hiz:.1f} m/s, '
            f'aralarinda {self.aralik:.1f} m')

    def _simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tik(self):
        t = self._simdi() - self.t0
        for i, ad in enumerate(self.adlar):
            s = (self.bas_s + i * self.aralik + self.hiz * t) % self.hat_boy
            x, y, teta = self.go.ofset_hat_nokta(
                s, self.ofset, self.duz, self.yaricap)

            st = EntityState()
            st.name = ad
            st.pose.position.x = float(x)
            st.pose.position.y = float(y)
            st.pose.position.z = 0.0
            st.pose.orientation.z = float(math.sin(teta / 2.0))
            st.pose.orientation.w = float(math.cos(teta / 2.0))
            # Hiz alani da doldurulursa Gazebo araci dogru yonde
            # hareketli gosterir (LiDAR/odom tutarliligi icin)
            st.twist.linear.x = float(self.hiz * math.cos(teta))
            st.twist.linear.y = float(self.hiz * math.sin(teta))
            st.reference_frame = 'world'

            req = SetEntityState.Request()
            req.state = st
            self.cli.call_async(req)     # yanit beklenmiyor: 30 Hz akis


def main(args=None):
    rclpy.init(args=args)
    try:
        d = Trafik()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(d)
    except KeyboardInterrupt:
        pass
    finally:
        d.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
