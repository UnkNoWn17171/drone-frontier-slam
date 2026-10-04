#!/usr/bin/env python3
# engel_kopru.py - lidar taramasini 72 sektore bolup ArduPilot'a OBSTACLE_DISTANCE olarak gonderir.
# ArduPilot'un kendi engelden kacinmasi (AP_Proximity) bu veriyle calisiyor.
#
# Kullanim (once baska terminalde: mav --out udp:127.0.0.1:14552):
#   python3 engel_kopru.py                        # udpin:127.0.0.1:14552
#   python3 engel_kopru.py --port /dev/ttyACM0    # gercek donanim
#   python3 engel_kopru.py --tani                 # gondermez, sadece olcer
#
# 14551 kesif.py'nin portu, ikisi ayni porta baglanamiyor.

import argparse
import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil

SEKTOR_SAYISI = 72          # OBSTACLE_DISTANCE en fazla 72 sektor aliyor
SEKTOR_GENISLIGI = 5.0      # derece
GONDERME_HZ = 10.0          # lidar 9 Hz civari, ArduPilot 500 ms veri gelmezse sensoru yok sayiyor

MESAFE_MIN_CM = 20          # daha yakini gurultu
MESAFE_MAX_CM = 1000        # lidar menzili 10 m
GECERSIZ = 65535            # sektorde olcum yok

VARSAYILAN_ADRES = "udpin:127.0.0.1:14552"


class EngelKopru(Node):
    def __init__(self, adres, tani_modu):
        super().__init__("engel_kopru")

        self.tani_modu = tani_modu
        self.sektorler = [GECERSIZ] * SEKTOR_SAYISI
        self.kilit = threading.Lock()
        self.tarama_sayaci = 0
        self.gonderim_sayaci = 0

        # /scan best effort yayinlaniyor, sensor QoS'u olmadan mesaj gelmiyor
        self.create_subscription(
            LaserScan, "/scan", self.tarama_geldi, qos_profile_sensor_data
        )

        if self.tani_modu:
            self.baglanti = None
            self.get_logger().info("TANI MODU, MAVLink'e hicbir sey gonderilmiyor")
        else:
            self.get_logger().info(f"MAVLink baglantisi kuruluyor: {adres}")
            self.baglanti = mavutil.mavlink_connection(
                adres, source_system=253
            )
            # yanlis portta sessizce beklemesin diye 5 sn'de bir uyari
            while self.baglanti.wait_heartbeat(timeout=5) is None:
                self.get_logger().warn(
                    f"heartbeat bekleniyor ({adres}). 'mav --out "
                    f"udp:127.0.0.1:14552' acik mi?"
                )
            self.get_logger().info(
                f"Baglandi: sistem {self.baglanti.target_system}, "
                f"bilesen {self.baglanti.target_component}"
            )

        self.create_timer(1.0 / GONDERME_HZ, self.gonder)
        self.create_timer(2.0, self.durum_yaz)

    def tarama_geldi(self, msg):
        """Her taramada sektorleri yeniden hesaplar."""
        yeni = [GECERSIZ] * SEKTOR_SAYISI

        for i, mesafe in enumerate(msg.ranges):
            if not math.isfinite(mesafe):
                continue
            if mesafe < msg.range_min or mesafe > msg.range_max:
                continue

            cm = int(mesafe * 100)
            if cm < MESAFE_MIN_CM or cm > MESAFE_MAX_CM:
                continue

            # ROS'ta aci saat yonunun tersine, MAVLink'te saat yonunde artiyor
            ros_aci = msg.angle_min + i * msg.angle_increment
            mav_derece = (-math.degrees(ros_aci)) % 360.0

            sektor = int(mav_derece / SEKTOR_GENISLIGI + 0.5) % SEKTOR_SAYISI

            # sektordeki en yakin olcum
            if cm < yeni[sektor]:
                yeni[sektor] = cm

        with self.kilit:
            self.sektorler = yeni
            self.tarama_sayaci += 1

    def gonder(self):
        """Sektorleri OBSTACLE_DISTANCE olarak gonderir."""
        if self.tani_modu or self.baglanti is None:
            return

        with self.kilit:
            veri = list(self.sektorler)

        # tarama gelmeden gonderilen bos mesaj etraf bos sanilir
        if self.tarama_sayaci == 0:
            return

        self.baglanti.mav.obstacle_distance_send(
            0,                                          # time_usec, 0 ise otopilotun saati
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            veri,                                       # 72 mesafe, cm
            0,                                          # increment, increment_f kullaniliyor
            MESAFE_MIN_CM,
            MESAFE_MAX_CM,
            float(SEKTOR_GENISLIGI),                    # increment_f, derece
            0.0,                                        # ilk sektor tam ileri
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
        )
        self.gonderim_sayaci += 1

    def durum_yaz(self):
        with self.kilit:
            veri = list(self.sektorler)
            tarama = self.tarama_sayaci

        gecerli = [d for d in veri if d != GECERSIZ]

        if not gecerli:
            self.get_logger().warn(
                f"tarama={tarama}, hicbir sektorde gecerli olcum yok "
                "(/scan geliyor mu, menzil dogru mu?)"
            )
            return

        en_yakin = min(gecerli)
        sektor = veri.index(en_yakin)

        # sektor 0 ileri, 18 sag, 36 arka, 54 sol
        def yon(idx):
            d = veri[idx]
            return "yok" if d == GECERSIZ else f"{d/100:.2f}"

        self.get_logger().info(
            f"tarama={tarama} gonderim={self.gonderim_sayaci} | "
            f"dolu={len(gecerli)}/72 | en yakin={en_yakin/100:.2f} m @ {sektor*5} derece"
        )
        self.get_logger().info(
            f"    ileri={yon(0)}  sag={yon(18)}  arka={yon(36)}  sol={yon(54)}  (metre)"
        )

        # tani modunda butun sektorleri bas
        if self.tani_modu:
            satir = []
            for i in range(0, SEKTOR_SAYISI, 12):
                parca = " ".join(
                    ("  --" if veri[j] == GECERSIZ else f"{veri[j]/100:5.2f}")
                    for j in range(i, min(i + 12, SEKTOR_SAYISI))
                )
                satir.append(f"    {i*5:3d}-{(min(i+11, SEKTOR_SAYISI-1))*5:3d} derece: {parca}")
            self.get_logger().info("tam profil:\n" + "\n".join(satir))


def main():
    ayrist = argparse.ArgumentParser()
    ayrist.add_argument("--port", default=VARSAYILAN_ADRES,
                        help="MAVLink adresi (varsayilan %(default)s; "
                             "14551 kesif.py'nin, kullanma)")
    ayrist.add_argument("--tani", action="store_true",
                        help="MAVLink'e gonderme, sadece olcumu ekrana yaz")
    arg = ayrist.parse_args()

    rclpy.init()
    dugum = None
    try:
        dugum = EngelKopru(arg.port, arg.tani)
        rclpy.spin(dugum)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C'yi rclpy yakalarsa ExternalShutdownException geliyor
        pass
    finally:
        if dugum is not None:
            dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
