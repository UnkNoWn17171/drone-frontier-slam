"""
harita_kalite.py - kaydedilen haritayi dunyanin gercek geometrisiyle karsilastirir

Dunyanin .sdf dosyasindan lidar duzlemini kesen yuzeyleri cikarir ve
haritayla karsilastirir.

Kullanim:
    python3 harita_kalite.py deneme --sdf ../ev_sim/ev.sdf
    python3 harita_kalite.py harita1 harita2 --sdf ...   (ayni dunyanin iki haritasi)
    python3 harita_kalite.py /baska/yer/harita.pgm --sdf ...   (yol da verilebilir)

Sadece <box> (yaw dahil) ve <cylinder> okunuyor, digerleri uyariyla atlaniyor.

Cikan sayilar (Oklid mesafesi):
  HASSASIYET  haritadaki dolu hucrelerin gercek yuzeye uzakligi
  KAPSAMA     gercek yuzeyin yuzde kaci haritada (TOLERANS_M icinde)
  KALINLIK    dolu hucre sayisi / gercek yuzey hucre sayisi
  NESNE       her nesnenin kendi yuzeyinin kapsamasi
  SIZINTI     tarama duzleminin altindaki esyalar haritaya girmis mi

map cercevesinin Gazebo cercevesiyle ayni oldugu varsayiliyor, yani drone
orijinde ve burnu +x'e bakarak dogmus olmali.
"""

import argparse
import math
import os
import re
import sys

from collections import deque

import numpy as np


HARITA_KLASOR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "haritalar")

# lidarin zeminden yuksekligi, drone yerdeyken (0.195 + 0.075)
LIDAR_OFSET = 0.27

# kapsama toleransi. EPS sart, 3 * 0.05 kayan noktada 0.15'ten biraz buyuk cikiyor
TOLERANS_M = 0.15
EPS = 1e-9

# nesne yuzeyinin en az bu kadari haritadaysa GORULDU
GORULDU_ORANI = 0.5

# gercek yuzeyden bundan uzak dolu hucre fazla sayiliyor
FAZLA_M = 0.5


def sdf_sekilleri(yol):
    """sdf'teki kutu ve silindirleri sozluk listesi olarak dondurur.

    box: ad, tip, x, y, z, yaw, sx, sy, sz
    cyl: ad, tip, x, y, z, yaw, r, sz
    Ikinci deger taninmayan geometrili link adlari.
    """
    with open(yol) as f:
        metin = f.read()
    sekiller = []
    atlanan = []
    for m in re.finditer(r'<link\s+name="([^"]+)"(.*?)</link>', metin, re.S):
        ad = m.group(1)
        govde = m.group(2)
        pm = re.search(r"<pose>([^<]+)</pose>", govde)
        if not pm:
            continue
        poz = [float(v) for v in pm.group(1).split()]
        while len(poz) < 6:
            poz.append(0.0)          # pose 3 sayi da olabilir
        bm = re.search(r"<box>\s*<size>([^<]+)</size>\s*</box>", govde, re.S)
        cm = re.search(r"<cylinder>\s*<radius>([^<]+)</radius>\s*"
                       r"<length>([^<]+)</length>\s*</cylinder>", govde, re.S)
        if bm:
            boy = [float(v) for v in bm.group(1).split()]
            if len(boy) < 3:
                atlanan.append(ad)
                continue
            sekiller.append({"ad": ad, "tip": "box",
                             "x": poz[0], "y": poz[1], "z": poz[2],
                             "yaw": poz[5],
                             "sx": boy[0], "sy": boy[1], "sz": boy[2]})
        elif cm:
            sekiller.append({"ad": ad, "tip": "cyl",
                             "x": poz[0], "y": poz[1], "z": poz[2],
                             "yaw": poz[5],
                             "r": float(cm.group(1)),
                             "sz": float(cm.group(2))})
        else:
            atlanan.append(ad)
    return sekiller, atlanan


def gorunur_sekiller(sekiller, tarama_z):
    """Tarama duzlemini gercekten kesen sekiller ve kesmeyenler."""
    gorunur = []
    gizli = []
    for s in sekiller:
        alt = s["z"] - s["sz"] / 2.0
        ust = s["z"] + s["sz"] / 2.0
        if alt <= tarama_z <= ust:
            gorunur.append(s)
        else:
            gizli.append((s, alt, ust))
    return gorunur, gizli


def _hucre(x, y, res, ox, oy):
    """Metreden (sutun, satir)'a, floor ile."""
    return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))


def ic_maske(gorunur, res, ox, oy, sekil):
    """Sekillerin icinde kalan hucreleri isaretler (hucre merkezine bakarak)."""
    h, w = sekil
    ic = np.zeros((h, w), dtype=bool)
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    for s in gorunur:
        dx = X - s["x"]
        dy = Y - s["y"]
        if s["tip"] == "cyl":
            ic |= (dx * dx + dy * dy) <= s["r"] * s["r"]
        else:
            # hucreyi seklin kendi cercevesine dondur
            c = math.cos(s["yaw"])
            sn = math.sin(s["yaw"])
            lx = c * dx + sn * dy
            ly = -sn * dx + c * dy
            # yari acik aralik ve eps, hucre merkezi duvar sinirina denk gelince float hatasi olmasin
            eps = res * 1e-6
            ic |= ((lx >= -s["sx"] / 2.0 - eps) & (lx < s["sx"] / 2.0 - eps) &
                   (ly >= -s["sy"] / 2.0 - eps) & (ly < s["sy"] / 2.0 - eps))
    return ic


def _pgm_oku_ham(yol):
    """P5 pgm okur, baslikta # yorumu olabilir."""
    with open(yol, "rb") as f:
        ham = f.read()
    parca = []
    i = 0
    while len(parca) < 4:
        while i < len(ham) and ham[i:i + 1].isspace():
            i += 1
        if ham[i:i + 1] == b"#":
            while i < len(ham) and ham[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(ham) and not ham[j:j + 1].isspace():
            j += 1
        if j == i:
            raise ValueError("pgm basligi eksik: " + yol)
        parca.append(ham[i:j])
        i = j
    if parca[0] != b"P5":
        raise ValueError("sadece P5 (ikili) pgm destekleniyor: " + yol)
    w, h, azami = int(parca[1]), int(parca[2]), int(parca[3])
    if azami > 255:
        raise ValueError("16 bit pgm desteklenmiyor: " + yol)
    i += 1                      # basligi bitiren bosluk
    return np.frombuffer(ham[i:i + w * h], dtype=np.uint8).reshape(h, w)


def harita_yollari(ad):
    """'deneme' icin haritalar/deneme.pgm ve .yaml, yol verilirse o yol."""
    ad = os.path.expanduser(ad)
    if ad.endswith((".pgm", ".yaml")) or os.sep in ad:
        temel = os.path.splitext(ad)[0]
    else:
        temel = os.path.join(HARITA_KLASOR, ad)
    return temel + ".pgm", temel + ".yaml"


def pgm_oku(ad):
    pgm, yaml_yolu = harita_yollari(ad)
    pikseller = _pgm_oku_ham(pgm)

    res = 0.05
    ox = oy = 0.0
    with open(yaml_yolu) as f:
        satirlar = f.readlines()
    for satir in satirlar:
        anahtar, _, deger = satir.partition(":")
        anahtar = anahtar.strip()
        if anahtar == "resolution":
            res = float(deger)
        elif anahtar == "origin":
            sayilar = deger.strip().strip("[]").split(",")
            ox, oy = float(sayilar[0]), float(sayilar[1])
    # pgm'de ilk satir en ustte, haritada y yukari artiyor
    dolu = np.flipud(pikseller < 50)
    return dolu, res, ox, oy


def gercek_yuzey(gorunur, res, ox, oy, sekil, ic_nokta=(0.0, 0.0)):
    """Iceriden gorulebilecek yuzey hucrelerini isaretler.

    Dis duvarlarin dis yuzu sayilmiyor. ic_nokta'dan bos alan doldurulup
    sadece o alana komsu yuzeyler aliniyor.
    """
    h, w = sekil
    ic = ic_maske(gorunur, res, ox, oy, sekil)

    # dolu bolgenin kenari
    kenar = np.zeros_like(ic)
    kenar[1:, :] |= ic[1:, :] & ~ic[:-1, :]
    kenar[:-1, :] |= ic[:-1, :] & ~ic[1:, :]
    kenar[:, 1:] |= ic[:, 1:] & ~ic[:, :-1]
    kenar[:, :-1] |= ic[:, :-1] & ~ic[:, 1:]

    # icerideki bos alani doldur
    icerisi = np.zeros((h, w), dtype=bool)
    c0, r0 = _hucre(ic_nokta[0], ic_nokta[1], res, ox, oy)
    if 0 <= r0 < h and 0 <= c0 < w and not ic[r0, c0]:
        icerisi[r0, c0] = True
        kuyruk = deque([(c0, r0)])
        while kuyruk:
            c, r = kuyruk.popleft()
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if 0 <= nr < h and 0 <= nc < w:
                    if not ic[nr, nc] and not icerisi[nr, nc]:
                        icerisi[nr, nc] = True
                        kuyruk.append((nc, nr))

    komsu_ic = np.zeros((h, w), dtype=bool)
    komsu_ic[1:, :] |= icerisi[:-1, :]
    komsu_ic[:-1, :] |= icerisi[1:, :]
    komsu_ic[:, 1:] |= icerisi[:, :-1]
    komsu_ic[:, :-1] |= icerisi[:, 1:]

    return kenar & komsu_ic, ic


def en_yakin_mesafe(kaynak, hedef, res):
    """kaynak'taki her hucrenin hedef'teki en yakin hucreye uzakligi (m).

    Sira np.argwhere(kaynak) sirasi.
    """
    ks = np.argwhere(kaynak).astype(np.float64)
    hs = np.argwhere(hedef).astype(np.float64)
    if len(ks) == 0:
        return np.zeros(0)
    if len(hs) == 0:
        return np.full(len(ks), np.inf)
    sonuc = np.empty(len(ks))
    blok = 512                   # bellek icin parca parca
    for i in range(0, len(ks), blok):
        k = ks[i:i + blok]
        d2 = ((k[:, None, :] - hs[None, :, :]) ** 2).sum(axis=2)
        sonuc[i:i + blok] = np.sqrt(d2.min(axis=1))
    return sonuc * res


def _ozet(d):
    """(ortalama, %95, en kotu), bos dizide nan."""
    if len(d) == 0:
        return float("nan"), float("nan"), float("nan")
    return float(d.mean()), float(np.percentile(d, 95)), float(d.max())


def degerlendir(ad, sdf, irtifa, ayrintili=True):
    dolu, res, ox, oy = pgm_oku(ad)
    tarama_z = irtifa + LIDAR_OFSET
    sekiller, atlanan = sdf_sekilleri(sdf)
    if atlanan:
        print("UYARI: taninmayan geometri, olcume girmedi: %s"
              % ", ".join(atlanan))
    gorunur, gizli = gorunur_sekiller(sekiller, tarama_z)
    yuzey, ic = gercek_yuzey(gorunur, res, ox, oy, dolu.shape)
    if yuzey.sum() == 0:
        print("UYARI: gorulebilir yuzey bulunamadi. Ic nokta (0,0) duvarin "
              "icine dusmus olabilir.")

    n_harita = int(dolu.sum())
    n_yuzey = int(yuzey.sum())

    d_harita = en_yakin_mesafe(dolu, yuzey, res)      # harita -> gercek
    d_yuzey = en_yakin_mesafe(yuzey, dolu, res)       # gercek -> harita
    ortalama, p95, en_kotu = _ozet(d_harita)
    kapsandi = d_yuzey <= TOLERANS_M + EPS
    kapsama = 100.0 * kapsandi.sum() / max(1, n_yuzey)
    n_fazla = int((d_harita > FAZLA_M).sum())
    kalinlik = float(n_harita) / max(1, n_yuzey)

    kapsandi_izgara = np.zeros_like(yuzey)
    kapsandi_izgara[yuzey] = kapsandi

    print("=" * 62)
    print("HARITA: %s   (tarama duzlemi %.2f m)" % (ad, tarama_z))
    print("=" * 62)
    print("harita %dx%d @ %.3f m, orijin (%.2f, %.2f)"
          % (dolu.shape[1], dolu.shape[0], res, ox, oy))
    print("dolu hucre %d | gercek yuzey hucresi %d" % (n_harita, n_yuzey))
    print()
    print("HASSASIYET  ortalama %.3f m | %%95 %.3f m | en kotu %.2f m"
          % (ortalama, p95, en_kotu))
    print("FAZLA       gercek yuzeyden %.1f m'den uzak %d hucre"
          % (FAZLA_M, n_fazla))
    print("KAPSAMA     gercek yuzeyin %%%.1f'i haritada (%.0f cm tolerans)"
          % (kapsama, TOLERANS_M * 100))
    print("KALINLIK    %.2f  (1.0 = keskin duvar, 2.0 = bulanik)" % kalinlik)

    goruldu = 0
    nesne_satirlari = []
    for s in gorunur:
        kendi = yuzey & ic_maske([s], res, ox, oy, dolu.shape)
        n = int(kendi.sum())
        etiket = s["tip"]
        if s["tip"] == "box" and abs(s["yaw"]) > 1e-6:
            etiket = "box %.0f deg" % math.degrees(s["yaw"])
        if n == 0:
            nesne_satirlari.append("   %-20s %-10s yuzeyi iceriden gorunmuyor"
                                   % (s["ad"], etiket))
            continue
        oran = float(kapsandi_izgara[kendi].sum()) / n
        if oran >= GORULDU_ORANI:
            goruldu += 1
        nesne_satirlari.append("   %-20s %-10s %s %-4s (%d yuzey hucresi)"
                               % (s["ad"], etiket,
                                  "GORULDU" if oran >= GORULDU_ORANI else "KACTI  ",
                                  "%%%d" % round(oran * 100), n))

    # sizinti: duzlemin altindaki esyanin 10 cm yakininda olup gorunur bir
    # yuzeye yakin olmayan dolu hucreler
    sizinti_satirlari = []
    toplam_sizinti = 0
    for (s, alt, ust) in gizli:
        if alt > tarama_z:
            sizinti_satirlari.append("   %-20s yukseklik %.2f - %.2f m  (duzlemin ustunde)"
                                     % (s["ad"], alt, ust))
            continue
        d_esya = en_yakin_mesafe(dolu, ic_maske([s], res, ox, oy, dolu.shape), res)
        n = int(((d_esya <= 0.10 + EPS) & (d_harita > TOLERANS_M + EPS)).sum())
        toplam_sizinti += n
        sizinti_satirlari.append("   %-20s yukseklik %.2f - %.2f m  %s"
                                 % (s["ad"], alt, ust,
                                    ("HARITADA: %d hucre" % n) if n else "yok (dogru)"))

    print("NESNE       %d / %d gorunur nesne GORULDU (kendi yuzeyinin >= %%%.0f'i)"
          % (goruldu, len(gorunur), GORULDU_ORANI * 100))
    print("SIZINTI     duzlemin altindaki esyalardan haritaya giren %d hucre"
          % toplam_sizinti)

    if ayrintili:
        print()
        print("NESNE DOKUMU (gorunmesi gerekenler):")
        for satir in nesne_satirlari:
            print(satir)
        print()
        print("GORUNMEMESI GEREKENLER (haritada olmamali):")
        for satir in sizinti_satirlari:
            print(satir)
    print()
    return {"hassasiyet": ortalama, "kapsama": kapsama, "kalinlik": kalinlik,
            "goruldu": goruldu, "nesne": len(gorunur), "fazla": n_fazla,
            "sizinti": toplam_sizinti}


def main():
    p = argparse.ArgumentParser(description="Harita kalitesini olc")
    p.add_argument("haritalar", nargs="+",
                   help="harita adi (uzantisiz, haritalar/ icinde) ya da .pgm yolu")
    p.add_argument("--irtifa", type=float, default=1.2,
                   help="o harita hangi ucus irtifasinda cikarildi")
    p.add_argument("--sdf", required=True, help="dunyanin .sdf dosyasi")
    a = p.parse_args()

    sdf = os.path.expanduser(a.sdf)
    if not os.path.exists(sdf):
        print("sdf bulunamadi: " + sdf)
        return 1

    sonuclar = {}
    for ad in a.haritalar:
        try:
            sonuclar[ad] = degerlendir(ad, sdf, a.irtifa,
                                       ayrintili=(len(a.haritalar) == 1))
        except (OSError, ValueError) as e:
            print("okunamadi: %s (%s)" % (ad, e))

    if len(sonuclar) > 1:
        print("=" * 62)
        print("KIYAS")
        print("%-16s %10s %10s %10s %8s %8s"
              % ("harita", "hassasiyet", "kapsama", "kalinlik", "nesne", "sizinti"))
        for ad, s in sonuclar.items():
            print("%-16s %9.3f m %9.1f%% %10.2f %4d/%-3d %8d"
                  % (ad, s["hassasiyet"], s["kapsama"], s["kalinlik"],
                     s["goruldu"], s["nesne"], s["sizinti"]))
    return 0 if sonuclar else 1


if __name__ == "__main__":
    sys.exit(main())
