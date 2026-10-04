"""
kesif.py - otonom kesif gorevi

drone.py ile harita.py'yi birlestirir. Drone kalkar, haritadaki bilinmeyen
alanin sinirina ucar, gidecek sinir kalmayinca kalkis noktasina donup iner.

Calistirma:
    python3 kesif.py --kaydet deneme
Secenekler:
    --irtifa 1.2      ucus irtifasi (metre)
    --sure 600        en fazla gorev suresi (saniye)
    --kaydet ad       bitince haritayi bu isimle kaydet
    --kayit-yok       harita kaydini atla
    --tani            ucma, sadece durum dokumu bas

Iki koordinat sistemi var. map Cartographer'in cercevesi (kalkis aninda
x ileri, y sol), NED ArduPilot'un cercevesi (x kuzey, y dogu, z asagi).
Aralarindaki iliski kalkistaki yaw acisina (psi0) bagli:
    ekf_x = cos(psi0)*map_x + sin(psi0)*map_y
    ekf_y = sin(psi0)*map_x - cos(psi0)*map_y

EKF'ye giden konumla planin yapildigi map cercevesi zamanla birbirinden
kayiyor. O yuzden oteleme her turda yeniden hesaplaniyor.
"""

import argparse
import math
import os
import sys
import threading
import time

from drone import Drone, DroneHatasi
import harita as H
import yol as Y


# iki tur arasinda otelemenin en fazla sicrayabilecegi mesafe (m)
SICRAMA_SINIRI = 0.3

# SLAM yaw'i ile EKF yaw'i bu kadar ayrisirsa gorev iptal (derece)
ACI_SINIRI_DERECE = 15.0

# pervane ucu govde merkezinden 0.3555 m, ustune yarim hucre. sisirme bunun altina inmez
ASGARI_SISIRME = 0.38

# sinir kalmadi denince harita guncellensin diye birkac kez daha bakiyoruz
BITIS_DENEME = 3

# kalkmadan iptal olduysa bos haritayi kaydetmiyoruz
uctu = False

# inisten once alinan harita. inerken lidar alcaktaki esyalari da gordugu icin
# inisten sonraki harita kullanilmiyor
son_harita = None

PROJE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# bir turda en fazla bu kadar ucup yeniden planliyoruz (m)
UFUK_M = 2.0


class Donusum:
    """map cercevesinden EKF NED cercevesine donusum.

    Aci a = psi_ned + psi_map. psi_ned ATTITUDE'dan, psi_map TF'ten gelen yaw.
    """

    def __init__(self, a):
        self.a = a
        self.c = math.cos(a)
        self.s = math.sin(a)
        self.tx = 0.0
        self.ty = 0.0

    @staticmethod
    def aciyi_hesapla(psi_ned, psi_map):
        """Acinin ilk tahmini. Drone biraz hareket edince aciyi_olc ile duzeltiliyor."""
        return psi_ned + psi_map

    @staticmethod
    def aciyi_olc(dmap, dekf):
        """Aciyi iki cercevedeki konum farklarindan cozer.

        dex = cos a * dmx + sin a * dmy
        dey = sin a * dmx - cos a * dmy
        denklemleri cos a ve sin a icin cozuluyor. Donen (a, olcek), olcek
        1'den cok uzaksa iki cerceve arasinda dogrusal iliski yok demektir.
        """
        dmx, dmy = dmap
        dex, dey = dekf
        kare = dmx * dmx + dmy * dmy
        if kare < 1e-9:
            return None, None
        c = (dex * dmx - dey * dmy) / kare
        sn = (dex * dmy + dey * dmx) / kare
        olcek = math.hypot(c, sn)
        return math.atan2(sn, c), olcek

    def yon(self, mx, my):
        """Sadece dondurme, fark vektorleri icin."""
        return (self.c * mx + self.s * my,
                self.s * mx - self.c * my)

    def nokta(self, mx, my):
        dx, dy = self.yon(mx, my)
        return (dx + self.tx, dy + self.ty)

    def oteleme_hesapla(self, map_xy, ekf_xy):
        dx, dy = self.yon(map_xy[0], map_xy[1])
        return (ekf_xy[0] - dx, ekf_xy[1] - dy)

    def oteleme_kur(self, map_xy, ekf_xy):
        self.tx, self.ty = self.oteleme_hesapla(map_xy, ekf_xy)

    def sapma(self, map_xy, ekf_xy):
        tx, ty = self.oteleme_hesapla(map_xy, ekf_xy)
        return math.hypot(tx - self.tx, ty - self.ty)

    def aci_sapmasi(self, psi_ned, psi_map):
        """Canli hesaplanan aci ile kurulu aci arasindaki fark (radyan)."""
        fark = self.aciyi_hesapla(psi_ned, psi_map) - self.a
        return abs(math.atan2(math.sin(fark), math.cos(fark)))


def yaz(s):
    print("[kesif] " + s, flush=True)


def ros_baslat():
    # ROS burada yukleniyor, dosyanin geri kalani ROS olmadan da import edilebilsin
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from harita_dugumu import HaritaDugumu
    rclpy.init()
    dugum = HaritaDugumu()
    yurutucu = SingleThreadedExecutor()
    yurutucu.add_node(dugum)
    is_parcacigi = threading.Thread(target=yurutucu.spin, daemon=True)
    is_parcacigi.start()
    return dugum, yurutucu, is_parcacigi


def ros_kapat(dugum, yurutucu, is_parcacigi):
    # once spin durur, sonra iplik beklenir, en son dugum silinir.
    # ters sirada C++ tarafi "terminate called" diye cokuyor
    try:
        import rclpy
    except ImportError:
        return
    try:
        if yurutucu is not None:
            yurutucu.shutdown()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    if is_parcacigi is not None:
        is_parcacigi.join(timeout=3.0)
    try:
        if dugum is not None:
            dugum.destroy_node()
    except Exception:
        pass


def otelemeyi_tazele(don, robot_map, s):
    """Otelemeyi simdiki (map, ekf) ciftiyle yeniden kurar, sicramayi dondurur.

    Sicrama SICRAMA_SINIRI'ni gecerse SLAM konumu kacirmis demektir, DroneHatasi.
    """
    ekf = (s["x"], s["y"])
    sicrama = don.sapma(robot_map[:2], ekf)
    if sicrama > SICRAMA_SINIRI:
        raise DroneHatasi(
            "oteleme %.2f m sicradi (sinir %.2f). map<->NED donusumu yanlis "
            "ya da SLAM kaydi. map=(%.2f, %.2f) ekf=(%.2f, %.2f)"
            % (sicrama, SICRAMA_SINIRI, robot_map[0], robot_map[1],
               ekf[0], ekf[1]))
    don.oteleme_kur(robot_map[:2], ekf)
    return sicrama


def veri_bekle(dugum, zaman_asimi=30.0):
    """Harita ve TF gelene kadar bekler."""
    son = time.time() + zaman_asimi
    while time.time() < son:
        h = dugum.harita_al()
        r = dugum.robot_konumu()
        if h is not None and r is not None:
            return h, r
        time.sleep(0.3)
    return None, None


def harita_guncellemesi_bekle(dugum, adet=2, zaman_asimi=6.0):
    """Harita 'adet' kez guncellenene ya da sure dolana kadar bekler."""
    basla = dugum.harita_sayaci()
    son = time.time() + zaman_asimi
    while time.time() < son:
        if dugum.harita_sayaci() - basla >= adet:
            return True
        time.sleep(0.2)
    return False


def harita_kaydet(ad, harita):
    """Haritayi haritalar/<ad>.pgm ve .yaml olarak yazar."""
    if harita is None:
        yaz("kaydedilecek harita yok (hic /map gelmedi).")
        return
    klasor = os.path.join(PROJE, "haritalar")
    os.makedirs(klasor, exist_ok=True)
    yol = os.path.join(klasor, ad)
    izgara, res, ox, oy = harita
    try:
        H.harita_yaz(yol, izgara, res, ox, oy)
        yaz("harita kaydedildi: %s.pgm (%dx%d)"
            % (yol, izgara.shape[1], izgara.shape[0]))
    except Exception as e:
        yaz("harita kaydedilemedi: %s" % e)


def tani(d, dugum, ayarlar):
    """Ucmadan durum dokumu. Kalkis kontrolunu ve planlayiciyi kuru calistirir."""
    h, robot_map = veri_bekle(dugum)
    if h is None or h[0] is None or robot_map is None:
        yaz("harita ya da TF gelmedi.")
        return
    izgara, res, ox, oy = h
    s = d.durum()
    print(H.ozet(izgara, res, ox, oy, robot_map[:2], 0.6))
    print("ned yaw %.2f derece | map yaw %.2f derece"
          % (math.degrees(s["yaw"]), math.degrees(robot_map[2])))
    print("ekf konum  = (%.2f, %.2f, %.2f)  varyans %.4f"
          % (s["x"], s["y"], s["z"], s["ekf_varyans"]))
    wp_acc = d.parametre_oku("WP_ACC")
    wp_spd = d.parametre_oku("WP_SPD")
    if wp_acc is not None:
        print("WP_ACC = %.2f m/s2 -> komut edilen egim %.1f derece"
              % (wp_acc, math.degrees(math.atan(wp_acc / 9.81))))
    if wp_spd is not None:
        print("WP_SPD = %.2f m/s" % wp_spd)
    try:
        d.orijin_bekle(zaman_asimi=5.0)
        print("ekf orijini: kurulu")
    except DroneHatasi:
        print("ekf orijini: YOK -> kalkis reddedilir (birkac sn bekle)")
    engelsiz = H.cevre_engelsiz_mi(izgara, res, ox, oy, robot_map[:2], 0.6)
    print("kalkis kontrolu (0.6 m'de duvar yok mu): %s"
          % ("GECTI" if engelsiz else "KALDI"))

    yol, bilgi = H.plan_yap(izgara, res, ox, oy, robot_map[:2],
                            sisirme_m=ayarlar.sisirme,
                            en_az_kume=ayarlar.en_az_kume,
                            geri_cekilme_m=ayarlar.geri_cekilme)
    print("planlayici: sinir kumesi %d" % bilgi["kume_sayisi"])
    if yol is None:
        print("  yol yok -> %s" % (bilgi.get("sebep") or bilgi.get("hata")))
    else:
        print("  hedef boyu %.1f m, yol %.1f m, %d nokta"
              % (bilgi["hedef_boy_m"], bilgi["yol_uzunlugu_m"], len(yol)))
        a = Donusum.aciyi_hesapla(s["yaw"], robot_map[2])
        don = Donusum(a)
        don.oteleme_kur(robot_map[:2], (s["x"], s["y"]))
        print("cerceve acisi = %.2f derece" % math.degrees(a))
        for i, (mx, my) in enumerate(yol[:6]):
            ex, ey = don.nokta(mx, my)
            print("  %d: map(%+.2f, %+.2f) -> ekf(%+.2f, %+.2f)"
                  % (i, mx, my, ex, ey))


def gorev(d, dugum, irtifa, en_fazla_sure, ayarlar):
    global uctu, son_harita

    h, robot_map = veri_bekle(dugum)
    if h is None:
        raise DroneHatasi("harita ya da TF gelmedi. Cartographer calisiyor mu?")
    izgara, res, ox, oy = h
    yaz("harita hazir: %dx%d @ %.3f m/hucre" % (izgara.shape[1], izgara.shape[0], res))

    # cerceve donusumu
    s = d.durum()
    a = Donusum.aciyi_hesapla(s["yaw"], robot_map[2])
    don = Donusum(a)
    don.oteleme_kur(robot_map[:2], (s["x"], s["y"]))
    yaz("sisirme %.2f m (taban %.2f) | geri cekilme %.2f m | irtifa %.2f m"
        % (ayarlar.sisirme, ayarlar.asgari_sisirme, ayarlar.geri_cekilme,
           irtifa))
    yaz("cerceve acisi (ilk tahmin) = %.2f derece, oteleme (%.2f, %.2f)"
        % (math.degrees(a), don.tx, don.ty))
    ilk_map = (robot_map[0], robot_map[1])
    ilk_ekf = (s["x"], s["y"])
    ilk_t = (don.tx, don.ty)
    aci_olculdu = False

    # 2D lidarda haritayi bozan ivme. egim atan(WP_ACC/9.81) kadar oluyor ve
    # egilen tarama duzlemi duvari yanlis yerde goruyor
    wp_acc = d.parametre_oku("WP_ACC")
    if wp_acc is not None:
        egim = math.degrees(math.atan(wp_acc / 9.81))
        if wp_acc > 1.0:
            yaz("UYARI: WP_ACC = %.2f m/s2 -> %.1f derece egim. "
                "2D lidar icin yuksek. MAVProxy'de: param set WP_ACC 0.5"
                % (wp_acc, egim))
        else:
            yaz("WP_ACC = %.2f m/s2 -> %.1f derece egim. Uygun."
                % (wp_acc, egim))

    if not H.cevre_engelsiz_mi(izgara, res, ox, oy, robot_map[:2], 0.6):
        raise DroneHatasi(
            "kalkis noktasinin 0.6 m cevresinde DOLU hucre var. "
            "Drone duvara cok yakin. --tani ile bak.")

    # orijin kurulmadan NAV_TAKEOFF reddediliyor
    d.orijin_bekle()
    d.mod_ayarla("GUIDED")
    d.arm()
    d.kalk(irtifa)
    uctu = True

    # yaw kalkistaki degerde sabit kaliyor, yoksa her hedefte biraz kayiyor
    d.hedef_yaw = d.durum()["yaw"]
    yaz("yaw kilitlendi: %.2f derece" % math.degrees(d.hedef_yaw))

    baslangic_zamani = time.time()
    bos_tur = 0
    BOS_TUR_AZAMI = 3
    tur = 0

    while True:
        gecen = time.time() - baslangic_zamani
        if gecen > en_fazla_sure:
            yaz("gorev suresi doldu (%.0f sn). Iniliyor." % gecen)
            break

        h = dugum.harita_al()
        robot_map = dugum.robot_konumu()
        if h is None or robot_map is None:
            yaz("harita/TF anlik kayip, bekleniyor...")
            time.sleep(0.5)
            continue
        izgara, res, ox, oy = h

        s = d.durum()

        # drone yeterince ilerleyince aciyi konum farklarindan olcup ilk tahminin yerine koyuyoruz
        if not aci_olculdu:
            dmap = (robot_map[0] - ilk_map[0], robot_map[1] - ilk_map[1])
            if math.hypot(dmap[0], dmap[1]) >= 0.8:
                dekf = (s["x"] - ilk_ekf[0], s["y"] - ilk_ekf[1])
                olculen, olcek = Donusum.aciyi_olc(dmap, dekf)
                if olculen is None or olcek is None or abs(olcek - 1.0) > 0.25:
                    raise DroneHatasi(
                        "aci olculemedi: olcek %.3f (1.0 olmaliydi). map ile "
                        "EKF arasinda dogrusal iliski yok -- SLAM kaydi."
                        % (olcek if olcek is not None else float("nan")))
                fark = math.degrees(abs(math.atan2(
                    math.sin(olculen - don.a), math.cos(olculen - don.a))))
                yaz("aci OLCULDU: %.2f derece (ilk tahmin %.2f, fark %.2f, "
                    "olcek %.3f)" % (math.degrees(olculen),
                                     math.degrees(don.a), fark, olcek))
                don = Donusum(olculen)
                don.oteleme_kur(robot_map[:2], (s["x"], s["y"]))
                ilk_t = (don.tx, don.ty)
                aci_olculdu = True

        sicrama = otelemeyi_tazele(don, robot_map, s)
        if tur > 0:
            yaz("oteleme tazelendi: sicrama %.3f m, kalkistan beri kayma %.3f m"
                % (sicrama, math.hypot(don.tx - ilk_t[0], don.ty - ilk_t[1])))
        aci_sap = math.degrees(don.aci_sapmasi(s["yaw"], robot_map[2]))
        if aci_sap > ACI_SINIRI_DERECE:
            raise DroneHatasi(
                "cerceve acisi %.1f derece kaydi (sinir %.1f). SLAM yaw'i ile "
                "EKF yaw'i ayrisiyor." % (aci_sap, ACI_SINIRI_DERECE))

        yol_map, bilgi = H.plan_yap(izgara, res, ox, oy, robot_map[:2],
                                    sisirme_m=ayarlar.sisirme,
                                    en_az_kume=ayarlar.en_az_kume,
                                    geri_cekilme_m=ayarlar.geri_cekilme)
        if yol_map is None:
            if not bilgi.get("bitti"):
                yaz("planlayici durdu: %s" % bilgi.get("hata"))
                break
            # haritanin son taramalari islemesi zaman aliyor, bitti demeden once tekrar bak
            bitti_gercek = True
            for deneme in range(BITIS_DENEME):
                harita_guncellemesi_bekle(dugum, 2, 6.0)
                h2 = dugum.harita_al()
                r2 = dugum.robot_konumu()
                if h2 is None or r2 is None:
                    continue
                sisirme = ayarlar.sisirme
                if deneme == BITIS_DENEME - 1:
                    # son denemede sisirme biraz kucultuluyor ama asgari degerin altina inmiyor
                    sisirme = max(ayarlar.sisirme * 0.6, ayarlar.asgari_sisirme)
                    if sisirme < ayarlar.sisirme:
                        yaz("son deneme: sisirme %.2f m'ye dusuruldu "
                            "(fiziksel taban %.2f)"
                            % (sisirme, ayarlar.asgari_sisirme))
                y2, b2 = H.plan_yap(h2[0], h2[1], h2[2], h2[3], r2[:2],
                                    sisirme_m=sisirme,
                                    en_az_kume=ayarlar.en_az_kume,
                                    geri_cekilme_m=ayarlar.geri_cekilme)
                if y2 is not None:
                    yaz("bekledikten sonra yol bulundu (deneme %d)." % (deneme + 1))
                    yol_map, bilgi = y2, b2
                    izgara, res, ox, oy = h2
                    robot_map = r2
                    bitti_gercek = False
                    break
            if bitti_gercek:
                yaz("KESIF TAMAM: %s (%d deneme sonrasi dogrulandi)"
                    % (bilgi.get("sebep", ""), BITIS_DENEME))
                break

        tur += 1
        yaz("tur %d | sinir kumesi %d | hedef boyu %.1f m | yol %.1f m"
            % (tur, bilgi["kume_sayisi"], bilgi.get("hedef_boy_m", 0.0),
               bilgi.get("yol_uzunlugu_m", 0.0)))

        # UFUK_M kadar uc, sonra yeniden planla
        kat_edilen = 0.0
        onceki = robot_map
        uculdu = False
        for (mx, my) in H.ucus_noktalari(yol_map, robot_map[:2], res):
            ex, ey = don.nokta(mx, my)
            d.git(ex, ey, irtifa, tolerans=ayarlar.tolerans,
                  zaman_asimi=ayarlar.hedef_zaman_asimi)
            uculdu = True
            kat_edilen += math.hypot(mx - onceki[0], my - onceki[1])
            onceki = (mx, my)

            for u in d.uyarilari_al():
                if "EKF" in u or "Thrust" in u or "Failsafe" in u:
                    yaz("DIKKAT -> " + u)

            if kat_edilen >= UFUK_M:
                break

        if not uculdu:
            # yol tek noktaya kirpilmis, harita guncellenince yeniden dene
            bos_tur += 1
            yaz("yol tek noktaya kirpildi, yeniden planlanacak (%d/%d)"
                % (bos_tur, BOS_TUR_AZAMI))
            if bos_tur >= BOS_TUR_AZAMI:
                yaz("ust uste bos tur: duruluyor.")
                break
            harita_guncellemesi_bekle(dugum, 2, ayarlar.bekleme_azami)
            continue
        bos_tur = 0

        # planlamadan once haritanin yeni taramayla guncellenmesini bekle
        harita_guncellemesi_bekle(dugum, 2, ayarlar.bekleme_azami)

    # kalkis noktasina don. lidar 1.47 m'de taradigi icin masa, sandalye gibi
    # alcak esyalar haritada yok, altinin bos oldugunu bildigimiz tek yer kalkis noktasi.
    # gorev suresi dolsa bile donus yapiliyor
    if uctu:
        try:
            h = dugum.harita_al()
            r = dugum.robot_konumu()
            if h is None or r is None:
                yaz("UYARI: harita/TF yok, kalkis noktasina donulemedi.")
            else:
                otelemeyi_tazele(don, r, d.durum())
                kalan = math.hypot(r[0] - ilk_map[0], r[1] - ilk_map[1])
                if kalan < ayarlar.tolerans:
                    yaz("kalkis noktasindayiz (%.2f m), donus gerekmiyor."
                        % kalan)
                elif not Y.noktaya_uc(d, don, h, r[:2], ilk_map,
                                      sisirme_m=ayarlar.sisirme,
                                      irtifa=irtifa,
                                      tolerans=ayarlar.tolerans,
                                      zaman_asimi=ayarlar.hedef_zaman_asimi,
                                      bacak_m=1.5,
                                      etiket="donus (%.2f m)" % kalan,
                                      yaz=yaz):
                    yaz("UYARI: kalkis noktasina DONULEMEDI. Drone bulundugu "
                        "yere inecek, altini gozle kontrol et.")
        except DroneHatasi as e:
            yaz("UYARI: donus sirasinda hata: %s" % e)

        # harita inmeden once aliniyor, main() inisten sonra kaydediyor
        harita_guncellemesi_bekle(dugum, 1, 3.0)
        son_harita = dugum.harita_al()
        yaz("harita inisten once alindi.")

    yaz("gorev dongusu bitti, %d tur, %.0f sn" % (tur, time.time() - baslangic_zamani))


def main():
    global son_harita

    p = argparse.ArgumentParser(description="Otonom SLAM kesif gorevi")
    p.add_argument("--adres", default="udpin:127.0.0.1:14551")
    p.add_argument("--irtifa", type=float, default=1.2)
    p.add_argument("--sure", type=float, default=600.0)
    p.add_argument("--sisirme", type=float, default=0.44,
                   help="engel sisirme yaricapi, metre "
                        "(0.44 = pervane ucu 0.3555 + hucre 0.025 + ekf 0.06)")
    p.add_argument("--tolerans", type=float, default=0.25,
                   help="hedefe varis toleransi, metre")
    p.add_argument("--en-az-kume", dest="en_az_kume", type=int, default=6,
                   help="bir sinir kumesinin en az kac hucre olmasi gerektigi")
    p.add_argument("--bekleme-azami", dest="bekleme_azami", type=float,
                   default=6.0,
                   help="harita guncellemesi icin en fazla bekleme, saniye")
    p.add_argument("--asgari-sisirme", dest="asgari_sisirme", type=float,
                   default=ASGARI_SISIRME,
                   help="sisirme bu degerin altina hicbir kosulda inmez")
    p.add_argument("--geri-cekilme", dest="geri_cekilme", type=float,
                   default=0.8,
                   help="sinira ne kadar kala durulacak, metre")
    p.add_argument("--hedef-zaman-asimi", dest="hedef_zaman_asimi",
                   type=float, default=60.0)
    p.add_argument("--kaydet", default="ev_oto",
                   help="bitince haritayi bu isimle kaydet")
    p.add_argument("--kayit-yok", dest="kayit_yok", action="store_true",
                   help="bitince haritayi kaydetme")
    p.add_argument("--tani", action="store_true",
                   help="ucma, sadece durum dokumu bas")
    ayarlar = p.parse_args()

    dugum = None
    yurutucu = None
    iplik = None
    d = None
    basarili = False
    try:
        dugum, yurutucu, iplik = ros_baslat()
        d = Drone(adres=ayarlar.adres)
        d.baslat()
        if ayarlar.tani:
            tani(d, dugum, ayarlar)
        else:
            gorev(d, dugum, ayarlar.irtifa, ayarlar.sure, ayarlar)
        basarili = True

    except DroneHatasi as e:
        yaz("GOREV IPTAL: %s" % e)
    except KeyboardInterrupt:
        yaz("kullanici durdurdu (Ctrl+C).")
    except Exception as e:
        yaz("BEKLENMEYEN HATA: %r" % e)

    finally:
        # nasil biterse bitsin drone havada kalmamali.
        # gorev hatayla bittiyse haritayi inmeden once al, kaydi inince yap
        if dugum is not None and uctu and son_harita is None:
            son_harita = dugum.harita_al()
        if d is not None:
            d.guvenli_kapat()
        if not ayarlar.kayit_yok and not ayarlar.tani and uctu:
            harita_kaydet(ayarlar.kaydet, son_harita)
        if d is not None:
            d.kapat()
        ros_kapat(dugum, yurutucu, iplik)
        yaz("kapandi.")
    return 0 if basarili else 1


if __name__ == "__main__":
    sys.exit(main())
