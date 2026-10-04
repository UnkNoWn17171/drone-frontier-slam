#!/usr/bin/env python3
# kol_ucus.py -- GUIDED + arm + takeoff, sonra LOITER'da Xbox kolu ile manuel ucus (Mode 2).
# Sol cubuk: gaz (yukari/asagi) + yaw | Sag cubuk: roll + pitch
# X: LAND (inis) | Y: LOITER. Inip disarm olunca cikar.
# Kayit icin: armdan sonra $KAYIT_DIR/kalkis_hazir dosyasini yazar, $KAYIT_DIR/kayit gelince takeoff verir.
import os
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame
from pymavlink import mavutil

ADRES = sys.argv[1] if len(sys.argv) > 1 else "udpin:127.0.0.1:14553"
KAYIT_DIR = os.environ.get("KAYIT_DIR", "")
IRTIFA = 1.2
IRTIFA_SINIR = 1.8
X, Y = 2, 3


def yaz(s):
    print(s, flush=True)


pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    yaz("[kol] HATA: kol bulunamadi")
    sys.exit(1)
kol = pygame.joystick.Joystick(0)
kol.init()
yaz(f"[kol] {kol.get_name()}")

# RC override'i ArduPilot yalnizca MAV_GCS_SYSID'den (255) kabul eder
m = mavutil.mavlink_connection(ADRES, source_system=255)
while True:   # MAVProxy'nin (GCS) heartbeat'ini atla, aracinkini bekle
    hb = m.recv_match(type="HEARTBEAT", blocking=True)
    if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
        m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        break
yaz("[kol] drone'a baglandi")
for msj_id, aralik in ((32, 100000), (65, 200000)):   # LOCAL_POSITION_NED, RC_CHANNELS
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msj_id, aralik, 0, 0, 0, 0, 0)

durum = {"armli": False, "mod": "", "irtifa": 0.0, "rc3": 0, "orijin": False}
sabit_gaz = 1100    # LOITER'a kadar sabit gaz (arm icin 1000: "Throttle (RC3) is not neutral"); None = gaz kolda
onceki = [0] * kol.get_numbuttons()
son_yaz = 0.0


def kanallar():
    pygame.event.pump()
    a = kol.get_axis
    return [int(1500 + a(3) * 500), int(1500 + a(4) * 500), int(1500 - a(1) * 500), int(1500 + a(0) * 500)]


def adim():
    """Kol -> RC override, gelen mesajlari isle. Her dongude cagrilir."""
    global onceki, son_yaz
    ch = kanallar()
    if sabit_gaz is not None:
        # LOITER'a kadar cubuklar ortada: arm, cubuk ortadan biraz kaysa "Yaw (RC4) is not neutral" diye reddediliyor
        ch = [1500, 1500, sabit_gaz, 1500]
    elif durum["irtifa"] > IRTIFA_SINIR and ch[2] > 1500:
        ch[2] = 1500   # duvarlar 2.5 m: ustune cikarsa 2D lidar duvari goremez, SLAM bozulur
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *ch, 0, 0, 0, 0)
    tus = [kol.get_button(i) for i in range(kol.get_numbuttons())]
    basildi = [t and not o for t, o in zip(tus, onceki)]
    onceki = tus
    if basildi[X]:
        yaz("[kol] X: LAND")
        m.set_mode("LAND")
    if basildi[Y]:
        yaz("[kol] Y: LOITER")
        m.set_mode("LOITER")
    while (msj := m.recv_match(blocking=False)) is not None:
        if msj.get_srcSystem() != m.target_system:
            continue
        tip = msj.get_type()
        if tip == "HEARTBEAT" and msj.type != mavutil.mavlink.MAV_TYPE_GCS:
            armli = bool(msj.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mod = mavutil.mode_string_v10(msj)
            if mod != durum["mod"]:
                durum["mod"] = mod
                yaz(f"[kol] mod: {mod}")
            if armli != durum["armli"]:
                durum["armli"] = armli
                yaz("[kol] armed." if armli else "[kol] disarm.")
        elif tip == "LOCAL_POSITION_NED":
            durum["irtifa"] = -msj.z
            if durum["armli"] and time.time() - son_yaz > 2:
                son_yaz = time.time()
                yaz(f"[kol] {durum['mod']} | irtifa {durum['irtifa']:4.2f} m")
        elif tip == "RC_CHANNELS":
            durum["rc3"] = msj.chan3_raw
        elif tip == "GPS_GLOBAL_ORIGIN":
            durum["orijin"] = True
        elif tip == "STATUSTEXT":
            yaz(f"[kol] AP: {msj.text}")
    time.sleep(0.04)


def bekle(kosul, sure):
    son = time.time() + sure
    while time.time() < son:
        adim()
        if kosul():
            return True
    return False


# 1) Override araca ulasiyor mu? (SITL'in kendi gazi 1000; 1100 gonderip RC3'te gor)
if not bekle(lambda: abs(durum["rc3"] - 1100) < 10, 10):
    yaz(f"[kol] HATA: kol araca ulasmiyor (RC3={durum['rc3']}). Kalkis yok.")
    sys.exit(1)
yaz(f"[kol] kol araca ulasiyor (RC3={durum['rc3']})")

sabit_gaz = 1000

# 2) EKF orijini: GPS yok, otopilot AHRS_ORIGIN_*'den kuruyor. Orijin yokken arm reddediliyor
son = time.time() + 60
while not durum["orijin"] and time.time() < son:
    m.mav.command_long_send(m.target_system, m.target_component,   # 49 = GPS_GLOBAL_ORIGIN
                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 49, 0, 0, 0, 0, 0, 0)
    bekle(lambda: durum["orijin"], 2)
if not durum["orijin"]:
    yaz("[kol] HATA: EKF orijini 60 sn'de kurulmadi. AHRS_OPTIONS=16 ve AHRS_ORIGIN_* yuklu mu?")
    sys.exit(1)
yaz("[kol] EKF orijini kurulu")

# 3) GUIDED + arm
m.set_mode("GUIDED")
if not bekle(lambda: durum["mod"] == "GUIDED", 10):
    yaz("[kol] HATA: GUIDED'a gecilemedi")
    sys.exit(1)
bekle(lambda: durum["rc3"] <= 1010, 5)
m.arducopter_arm()
if not bekle(lambda: durum["armli"], 15):
    yaz("[kol] HATA: arm olmadi")
    sys.exit(1)

# 4) Kayit baslasin, sonra takeoff
if KAYIT_DIR:
    open(os.path.join(KAYIT_DIR, "kalkis_hazir"), "w").close()
    bekle(lambda: os.path.exists(os.path.join(KAYIT_DIR, "kayit")), 5)
yaz(f"[kol] takeoff {IRTIFA} m")
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, IRTIFA)
if not bekle(lambda: durum["irtifa"] > IRTIFA - 0.15 or not durum["armli"], 20):
    yaz("[kol] UYARI: takeoff irtifasina ulasilamadi")

# 5) LOITER: kontrol kolda (gaz cubugu ortada = irtifayi tut)
sabit_gaz = None
m.set_mode("LOITER")
yaz("[kol] KOL SENDE -> sag cubuk: git | sol cubuk: irtifa + yaw | X: inis | Y: LOITER")
while durum["armli"]:
    adim()
yaz("[kol] indi ve disarm oldu. UCUS BITTI")
