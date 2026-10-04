"""
drone.py - MAVLink uzerinden otopilotla konusan kisim

ROS ve haritayla isi yok, sadece baglanma, kalkma, gitme ve inme.

Kullanim:
    d = Drone()
    d.baslat()
    d.mod_ayarla("GUIDED"); d.arm(); d.kalk(1.2)
    d.git(2.0, 0.0, 1.2)
    d.indir()
    d.kapat()
"""

import math
import threading
import time

from pymavlink import mavutil


# SET_POSITION_TARGET_LOCAL_NED type_mask. hiz, ivme ve yaw_rate yoksayiliyor,
# sadece konum ve yaw kullaniliyor (8+16+32 + 64+128+256 + 2048)
KONUM_VE_YAW = 2552

# SET_MESSAGE_INTERVAL icin mesaj id'leri
MSG_ATTITUDE = 30
MSG_LOCAL_POSITION_NED = 32
MSG_EKF_STATUS_REPORT = 193
MSG_GPS_GLOBAL_ORIGIN = 49

# ArduPilot'un EKF failsafe esigi 1.0, biz 0.5'te gorevi kesip kendimiz iniyoruz
EKF_VARYANS_SINIRI = 0.5


class DroneHatasi(Exception):
    """Ucusla ilgili hatalar. kesif.py bunu yakalayip iniyor."""


class Drone:

    def __init__(self, adres="udpin:127.0.0.1:14551", sysid=254, ayrintili=True):
        # MAVProxy 255 kullaniyor, ACK'leri ayirt edebilmek icin biz 254
        self.adres = adres
        self.ayrintili = ayrintili
        self.sysid = sysid
        self.baglanti = mavutil.mavlink_connection(adres, source_system=sysid)

        # otopilotun kimligi, baslat() ilk arac heartbeat'inden dolduruyor
        self._hedef_sys = None
        self._hedef_comp = None
        self._hb_comp = None
        self._arac_tipi = None

        self._kilit = threading.Lock()
        self._calisiyor = False
        self._okuyucu = None

        # okuma dongusunun guncelledigi durum, self._kilit ile okunuyor
        self._d = {
            "x": None, "y": None, "z": None,        # NED metre (z asagi pozitif)
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "yaw": None,                             # radyan, NED (0 = kuzey)
            "armed": False,
            "mod": None,
            "ekf_varyans": 0.0,
            "son_konum_ts": 0.0,
            "orijin": False,                         # EKF orijini kurulu mu
        }
        self._ackler = {}        # komut_id -> MAV_RESULT
        self._uyarilar = []      # STATUSTEXT satirlari
        self._parametreler = {}  # ad -> deger

        # None ise git() o anki yaw'i kullanir ve her hedefte biraz kayar.
        # kesif.py kalkistan sonra bunu bir kere ayarliyor
        self.hedef_yaw = None

    def _yaz(self, s):
        if self.ayrintili:
            print("[drone] " + s, flush=True)

    def durum(self):
        """Durumun anlik kopyasi. Disaridan hep bununla okunur."""
        with self._kilit:
            return dict(self._d)

    def uyarilari_al(self):
        """Biriken STATUSTEXT satirlarini alir ve listeyi bosaltir."""
        with self._kilit:
            u = list(self._uyarilar)
            self._uyarilar = []
        return u

    def baslat(self, zaman_asimi=15.0):
        self._yaz("baglaniyor: " + self.adres)
        hb = self._arac_heartbeat_bekle(zaman_asimi)
        if hb is None:
            raise DroneHatasi(
                "heartbeat gelmedi. Simulasyon kapali olabilir ya da port yanlis.")
        self._hedef_sys = hb.get_srcSystem()
        self._hedef_comp = hb.get_srcComponent()
        self._hb_comp = self._hedef_comp
        self._arac_tipi = hb.type
        # MAVProxy uzerinden compid 0 gelebiliyor, komutlari dogrudan otopilota (1) gonderiyoruz
        if self._hedef_comp == 0:
            self._hedef_comp = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        self.baglanti.target_system = self._hedef_sys
        self.baglanti.target_component = self._hedef_comp
        self._yaz("baglandi. sysid=%d compid=%d"
                  % (self._hedef_sys, self._hedef_comp))

        self._calisiyor = True
        self._okuyucu = threading.Thread(target=self._oku_dongu, daemon=True)
        self._okuyucu.start()

        self._mesaj_araligi(MSG_LOCAL_POSITION_NED, 10)
        self._mesaj_araligi(MSG_ATTITUDE, 5)
        self._mesaj_araligi(MSG_EKF_STATUS_REPORT, 2)

        if not self.veri_bekle(zaman_asimi=10.0):
            raise DroneHatasi(
                "LOCAL_POSITION_NED akmiyor. EKF henuz konum uretmiyor olabilir.")
        self._yaz("konum verisi akiyor.")

    def _arac_heartbeat_bekle(self, zaman_asimi):
        """Ilk arac heartbeat'ini bekler, MAVProxy gibi yer istasyonlarininkini atlar."""
        son = time.time() + zaman_asimi
        while time.time() < son:
            hb = self.baglanti.recv_match(type="HEARTBEAT", blocking=True,
                                          timeout=max(0.1, son - time.time()))
            if hb is None:
                continue
            if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            return hb
        return None

    def _bizim_mi(self, msg, tip):
        """Mesaj bizim otopilottan mi geliyor, ACK ise bize mi.

        Yoksa MAVProxy'nin heartbeat'i armed=False yazabilir, onun ACK'i de
        bizim komutun cevabi sanilabilir.
        """
        if tip == "BAD_DATA" or msg.get_srcSystem() != self._hedef_sys:
            return False
        if tip == "HEARTBEAT":
            return (msg.get_srcComponent() == self._hb_comp
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS)
        if tip == "COMMAND_ACK":
            # MAVLink1'de target_system alani yok, 0 sayiyoruz
            return getattr(msg, "target_system", 0) in (0, self.sysid)
        return True

    def _oku_dongu(self):
        while self._calisiyor:
            msg = self.baglanti.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            tip = msg.get_type()
            if not self._bizim_mi(msg, tip):
                continue

            if tip == "LOCAL_POSITION_NED":
                with self._kilit:
                    self._d["x"] = msg.x
                    self._d["y"] = msg.y
                    self._d["z"] = msg.z
                    self._d["vx"] = msg.vx
                    self._d["vy"] = msg.vy
                    self._d["vz"] = msg.vz
                    self._d["son_konum_ts"] = time.time()

            elif tip == "ATTITUDE":
                with self._kilit:
                    self._d["yaw"] = msg.yaw

            elif tip == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mod = None
                try:
                    mod = mavutil.mode_string_v10(msg)
                except Exception:
                    pass
                with self._kilit:
                    self._d["armed"] = armed
                    if mod:
                        self._d["mod"] = mod

            elif tip == "EKF_STATUS_REPORT":
                with self._kilit:
                    self._d["ekf_varyans"] = msg.pos_horiz_variance

            elif tip == "GPS_GLOBAL_ORIGIN":
                with self._kilit:
                    self._d["orijin"] = True

            elif tip == "PARAM_VALUE":
                ad = msg.param_id
                if isinstance(ad, bytes):
                    ad = ad.decode("ascii", "ignore")
                with self._kilit:
                    self._parametreler[ad.strip("\x00")] = msg.param_value

            elif tip == "COMMAND_ACK":
                with self._kilit:
                    self._ackler[msg.command] = msg.result

            elif tip == "STATUSTEXT":
                metin = msg.text
                if isinstance(metin, bytes):
                    metin = metin.decode("ascii", "ignore")
                metin = metin.strip()
                with self._kilit:
                    self._uyarilar.append(metin)
                self._yaz("AP: " + metin)

    def veri_bekle(self, zaman_asimi=10.0):
        son = time.time() + zaman_asimi
        while time.time() < son:
            d = self.durum()
            if d["x"] is not None and d["yaw"] is not None:
                return True
            time.sleep(0.1)
        return False

    def _komut(self, komut_id, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
               zaman_asimi=5.0, ack_bekle=True):
        """command_long gonderir ve COMMAND_ACK'i bekler, red gelirse hata verir."""
        with self._kilit:
            self._ackler.pop(komut_id, None)

        self.baglanti.mav.command_long_send(
            self._hedef_sys,
            self._hedef_comp,
            komut_id, 0, p1, p2, p3, p4, p5, p6, p7)

        if not ack_bekle:
            return True

        son = time.time() + zaman_asimi
        while time.time() < son:
            with self._kilit:
                sonuc = self._ackler.get(komut_id)
            if sonuc is not None:
                if sonuc == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                raise DroneHatasi("komut %d reddedildi, MAV_RESULT=%d"
                                  % (komut_id, sonuc))
            time.sleep(0.05)
        raise DroneHatasi("komut %d icin ACK gelmedi (%.1f sn)"
                          % (komut_id, zaman_asimi))

    def _mesaj_araligi(self, mesaj_id, hz):
        aralik_us = int(1e6 / hz)
        try:
            self._komut(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        p1=mesaj_id, p2=aralik_us, zaman_asimi=3.0)
        except DroneHatasi:
            # ACK gelmese de akis MAVProxy uzerinden geliyorsa sorun yok
            self._yaz("uyari: mesaj %d icin aralik ayarlanamadi" % mesaj_id)

    def parametre_oku(self, ad, zaman_asimi=5.0):
        """Tek bir ArduPilot parametresini okur. Bulamazsa None."""
        with self._kilit:
            self._parametreler.pop(ad, None)
        self.baglanti.mav.param_request_read_send(
            self._hedef_sys,
            self._hedef_comp,
            ad.encode("ascii"), -1)
        son = time.time() + zaman_asimi
        while time.time() < son:
            with self._kilit:
                if ad in self._parametreler:
                    return self._parametreler[ad]
            time.sleep(0.05)
        return None

    def mod_ayarla(self, ad, zaman_asimi=8.0):
        # baglantinin mode_mapping()'i son heartbeat'e bakiyor, o MAVProxy'ninki olabilir
        eslesme = mavutil.mode_mapping_byname(self._arac_tipi) or {}
        if ad not in eslesme:
            raise DroneHatasi("bilinmeyen mod: " + ad)
        self.baglanti.mav.set_mode_send(
            self._hedef_sys,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            eslesme[ad])

        son = time.time() + zaman_asimi
        while time.time() < son:
            if self.durum()["mod"] == ad:
                self._yaz("mod: " + ad)
                return True
            time.sleep(0.1)
        raise DroneHatasi("mod %s'a gecilemedi (su an: %s)"
                          % (ad, self.durum()["mod"]))

    def orijin_bekle(self, zaman_asimi=60.0):
        """EKF orijini kurulana kadar bekler, orijin yokken kalkis reddediliyor.

        GPS olmadigi icin orijini otopilot AHRS_ORIGIN_* ve AHRS_OPTIONS bit 4 ile kuruyor.
        """
        if self.durum()["orijin"]:
            return True
        self._yaz("EKF orijini bekleniyor...")
        son = time.time() + zaman_asimi
        sonraki_istek = 0.0
        while time.time() < son:
            if self.durum()["orijin"]:
                self._yaz("EKF orijini kurulu.")
                return True
            # orijin biz baglanmadan kurulduysa mesaj gelmez, o yuzden istiyoruz
            if time.time() >= sonraki_istek:
                self._komut(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                            p1=MSG_GPS_GLOBAL_ORIGIN, ack_bekle=False)
                sonraki_istek = time.time() + 2.0
            time.sleep(0.1)
        raise DroneHatasi(
            "EKF orijini %.0f sn'de kurulmadi; kalkis reddedilirdi. "
            "AHRS_OPTIONS=16 ve AHRS_ORIGIN_* yuklu mu?" % zaman_asimi)

    def arm(self, zaman_asimi=10.0):
        self._komut(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
        son = time.time() + zaman_asimi
        while time.time() < son:
            if self.durum()["armed"]:
                self._yaz("armed.")
                return True
            time.sleep(0.1)
        raise DroneHatasi("arm komutu kabul edildi ama motorlar armed olmadi.")

    def kalk(self, irtifa, zaman_asimi=45.0):
        self._komut(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=irtifa)
        self._yaz("kalkis: %.2f m" % irtifa)
        son = time.time() + zaman_asimi
        while time.time() < son:
            d = self.durum()
            if d["z"] is not None and (-d["z"]) >= irtifa * 0.95:
                self._yaz("irtifa tamam: %.2f m" % (-d["z"]))
                time.sleep(1.0)
                return True
            self._saglik_kontrol()
            time.sleep(0.2)
        raise DroneHatasi("kalkis irtifasina %.0f sn'de ulasilamadi" % zaman_asimi)

    def git(self, x, y, irtifa, tolerans=0.25, zaman_asimi=60.0):
        """NED cercevesinde (x, y) noktasina gider ve varmasini bekler.

        irtifa yukari pozitif veriliyor, NED'de z asagi oldugu icin -irtifa gonderiliyor.
        """
        if self.hedef_yaw is not None:
            yaw = self.hedef_yaw
        else:
            d0 = self.durum()
            yaw = d0["yaw"] if d0["yaw"] is not None else 0.0

        self._hedef_gonder(x, y, -irtifa, yaw)
        son = time.time() + zaman_asimi
        son_ilerleme = time.time()
        en_iyi = float("inf")

        while time.time() < son:
            self._saglik_kontrol(irtifa)
            d = self.durum()
            if d["x"] is None:
                time.sleep(0.1)
                continue

            mesafe = math.hypot(x - d["x"], y - d["y"])
            hiz = math.hypot(d["vx"], d["vy"])

            if mesafe < tolerans and hiz < 0.15:
                return True

            if mesafe < en_iyi - 0.05:
                en_iyi = mesafe
                son_ilerleme = time.time()
            elif time.time() - son_ilerleme > 4.0:
                # 4 sn ilerleme yoksa hedefi tekrar gonder
                self._hedef_gonder(x, y, -irtifa, yaw)
                son_ilerleme = time.time()

            time.sleep(0.1)

        raise DroneHatasi("hedefe varilamadi (%.2f, %.2f), kalan %.2f m"
                          % (x, y, en_iyi))

    def _hedef_gonder(self, x, y, z, yaw):
        self.baglanti.mav.set_position_target_local_ned_send(
            0,
            self._hedef_sys,
            self._hedef_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            KONUM_VE_YAW,
            x, y, z,          # konum, NED metre
            0, 0, 0,          # hiz (yoksayiliyor)
            0, 0, 0,          # ivme (yoksayiliyor)
            yaw, 0)           # yaw sabit, yaw_rate yoksayiliyor

    def _saglik_kontrol(self, hedef_irtifa=None):
        d = self.durum()
        if hedef_irtifa is not None and d["z"] is not None:
            if abs((-d["z"]) - hedef_irtifa) > 0.6:
                raise DroneHatasi(
                    "irtifa %.2f m, hedef %.2f m -- 2D lidar duzlemi kaydi."
                    % (-d["z"], hedef_irtifa))
        if d["ekf_varyans"] > EKF_VARYANS_SINIRI:
            raise DroneHatasi(
                "EKF konum varyansi %.3f (sinir %.2f) -- SLAM konumu kacirdi."
                % (d["ekf_varyans"], EKF_VARYANS_SINIRI))
        if d["son_konum_ts"] and time.time() - d["son_konum_ts"] > 3.0:
            raise DroneHatasi("3 sn'dir konum mesaji gelmiyor -- baglanti koptu.")

    def indir(self, zaman_asimi=90.0):
        """LAND moduna gecip disarm olmasini bekler. Hata firlatmaz."""
        try:
            self.mod_ayarla("LAND", zaman_asimi=5.0)
        except DroneHatasi as e:
            self._yaz("LAND moduna gecilemedi: %s" % e)
        son = time.time() + zaman_asimi
        while time.time() < son:
            if not self.durum()["armed"]:
                self._yaz("indi ve disarm oldu.")
                return True
            time.sleep(0.5)
        self._yaz("uyari: disarm gorulmedi.")
        return False

    def guvenli_kapat(self):
        """Drone havadaysa indirir. kesif.py'de finally blogundan cagriliyor."""
        try:
            d = self.durum()
            if d["armed"]:
                self._yaz("guvenli kapanis: iniliyor.")
                self.indir()
        except Exception as e:
            self._yaz("guvenli kapanista hata: %s" % e)

    def kapat(self):
        self._calisiyor = False
        if self._okuyucu is not None:
            self._okuyucu.join(timeout=2.0)
        try:
            self.baglanti.close()
        except Exception:
            pass


if __name__ == "__main__":
    # tek basina calistirilirsa sadece baglanti testi, ucmaz
    d = Drone()
    d.baslat()
    s = d.durum()
    print("konum NED: x=%.2f y=%.2f z=%.2f" % (s["x"], s["y"], s["z"]))
    print("yaw: %.1f derece" % math.degrees(s["yaw"]))
    print("mod: %s  armed: %s  ekf_varyans: %.4f"
          % (s["mod"], s["armed"], s["ekf_varyans"]))
    d.kapat()
