"""
harita.py - sinir bulma ve yol planlama

Izgarada bilinmeyen alanla bos alanin sinirini bulur ve oraya duvara
carpmadan giden yolu cikarir. ROS kullanmiyor, /map'i harita_dugumu.py okuyor.
Hesaplar map cercevesinde ve metre cinsinden.

Izgara degerleri (OccupancyGrid): -1 bilinmiyor, 0 bos, 100 dolu.
"""

import math
import os
from collections import deque

import numpy as np


# Cartographer'da bir kez bos gorulen hucre 0'a degil 42-49 civarina iniyor,
# o yuzden esik 50
BOS_ESIGI = 49       # 0..49 bos
DOLU_ESIGI = 50      # 50..100 dolu

KOMSU8 = ((1, 0), (-1, 0), (0, 1), (0, -1),
          (1, 1), (1, -1), (-1, 1), (-1, -1))

def hucreye(x, y, res, ox, oy):
    """Metreden (sutun, satir)'a. int() negatif tarafta yanlis hucre verdigi icin floor."""
    return (int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res)))


def metreye(sutun, satir, res, ox, oy):
    """Hucre merkezinin metre karsiligi."""
    return (ox + (sutun + 0.5) * res, oy + (satir + 0.5) * res)


def maskeler(izgara):
    """Bos, dolu ve bilinmiyor maskeleri."""
    bilinmiyor = izgara < 0
    bos = (izgara >= 0) & (izgara <= BOS_ESIGI)
    dolu = izgara >= DOLU_ESIGI
    return bos, dolu, bilinmiyor


def sisir(dolu, yaricap_hucre):
    """Engelleri yaricap kadar buyutur, drone nokta olmadigi icin."""
    if yaricap_hucre <= 0:
        return dolu.copy()
    h, w = dolu.shape
    sonuc = dolu.copy()
    r = int(yaricap_hucre)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            if dx == 0 and dy == 0:
                continue
            # diziyi (dx, dy) kadar kaydirip veya'la
            y0k, y1k = max(0, dy), min(h, h + dy)
            x0k, x1k = max(0, dx), min(w, w + dx)
            y0h, y1h = max(0, -dy), min(h, h - dy)
            x0h, x1h = max(0, -dx), min(w, w - dx)
            sonuc[y0k:y1k, x0k:x1k] |= dolu[y0h:y1h, x0h:x1h]
    return sonuc


def mesafe_alani(gecilebilir, baslangic):
    """Baslangictan her hucreye BFS mesafesi (hucre). Ulasilamayan hucre -1.

    Tek BFS ile butun sinir adaylarinin uzakligi birden cikiyor, yol da bu
    alanin uzerinden geri gidilerek kuruluyor.
    """
    h, w = gecilebilir.shape
    mes = np.full((h, w), -1, dtype=np.int32)
    sc, sr = baslangic
    if not (0 <= sr < h and 0 <= sc < w) or not gecilebilir[sr, sc]:
        return mes
    mes[sr, sc] = 0
    kuyruk = deque([(sc, sr)])
    while kuyruk:
        c, r = kuyruk.popleft()
        d = mes[r, c] + 1
        for dc, dr in KOMSU8:
            nc, nr = c + dc, r + dr
            if 0 <= nr < h and 0 <= nc < w:
                if gecilebilir[nr, nc] and mes[nr, nc] < 0:
                    mes[nr, nc] = d
                    kuyruk.append((nc, nr))
    return mes


def sinir_hucreleri(bos, bilinmiyor):
    """Sinir (frontier): bilinmeyene komsu bos hucreler."""
    h, w = bos.shape
    komsu_bilinmiyor = np.zeros((h, w), dtype=bool)
    komsu_bilinmiyor[:-1, :] |= bilinmiyor[1:, :]
    komsu_bilinmiyor[1:, :] |= bilinmiyor[:-1, :]
    komsu_bilinmiyor[:, :-1] |= bilinmiyor[:, 1:]
    komsu_bilinmiyor[:, 1:] |= bilinmiyor[:, :-1]
    return bos & komsu_bilinmiyor


def kumele(maske, en_az=6):
    """Bitisik sinir hucrelerini gruplar, en_az'dan kucuk gruplari atar."""
    h, w = maske.shape
    gorulen = np.zeros((h, w), dtype=bool)
    kumeler = []
    satirlar, sutunlar = np.nonzero(maske)
    for sr, sc in zip(satirlar, sutunlar):
        if gorulen[sr, sc]:
            continue
        kuyruk = deque([(sc, sr)])
        gorulen[sr, sc] = True
        uyeler = []
        while kuyruk:
            c, r = kuyruk.popleft()
            uyeler.append((c, r))
            for dc, dr in KOMSU8:
                nc, nr = c + dc, r + dr
                if 0 <= nr < h and 0 <= nc < w:
                    if maske[nr, nc] and not gorulen[nr, nc]:
                        gorulen[nr, nc] = True
                        kuyruk.append((nc, nr))
        if len(uyeler) >= en_az:
            kumeler.append(uyeler)
    return kumeler


def gorus_var_mi(gecilebilir, a, b):
    """a'dan b'ye duz cizgi gecilebilir mi (Bresenham)."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    hata = dx - dy
    h, w = gecilebilir.shape
    while True:
        if not (0 <= y0 < h and 0 <= x0 < w) or not gecilebilir[y0, x0]:
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * hata
        if e2 > -dy:
            hata -= dy
            x0 += sx
        if e2 < dx:
            hata += dx
            y0 += sy


def yol_kur(mes, gecilebilir, hedef):
    """Mesafe alaninda hedeften baslangica geri giderek yolu kurar."""
    h, w = mes.shape
    c, r = hedef
    if mes[r, c] < 0:
        return None
    yol = [(c, r)]
    while mes[r, c] > 0:
        en_iyi = None
        for dc, dr in KOMSU8:
            nc, nr = c + dc, r + dr
            if 0 <= nr < h and 0 <= nc < w and mes[nr, nc] >= 0:
                if mes[nr, nc] == mes[r, c] - 1:
                    en_iyi = (nc, nr)
                    break
        if en_iyi is None:
            return None
        c, r = en_iyi
        yol.append((c, r))
    yol.reverse()
    return yol


def yolu_kisalt(yol, gecilebilir):
    """Gorus olan ara noktalari atip yolu duz bacaklara indirir."""
    if not yol:
        return []
    sonuc = [yol[0]]
    i = 0
    while i < len(yol) - 1:
        j = len(yol) - 1
        while j > i + 1 and not gorus_var_mi(gecilebilir, yol[i], yol[j]):
            j -= 1
        sonuc.append(yol[j])
        i = j
    return sonuc


def geri_cekil(yol_m, mesafe_m):
    """Yolun son mesafe_m metresini kirpar. Yol daha kisaysa sadece ilk nokta kalir."""
    if mesafe_m <= 0 or len(yol_m) < 2:
        return list(yol_m)
    kalan = mesafe_m
    i = len(yol_m) - 1
    while i > 0:
        x1, y1 = yol_m[i]
        x0, y0 = yol_m[i - 1]
        d = math.hypot(x1 - x0, y1 - y0)
        if d >= kalan:
            t = (d - kalan) / d if d > 0 else 0.0
            yeni = (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            return yol_m[:i] + [yeni]
        kalan -= d
        i -= 1
    return yol_m[:1]


def bol(yol_m, en_fazla=1.5):
    """Uzun bacaklari en_fazla metrelik parcalara boler."""
    if len(yol_m) < 2:
        return list(yol_m)
    sonuc = [yol_m[0]]
    for i in range(1, len(yol_m)):
        x0, y0 = sonuc[-1]
        x1, y1 = yol_m[i]
        d = math.hypot(x1 - x0, y1 - y0)
        if d > en_fazla:
            n = int(d / en_fazla) + 1
            for k in range(1, n):
                t = float(k) / n
                sonuc.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        sonuc.append((x1, y1))
    return sonuc


# hedef geri cekilme mesafesinden en az bu kadar uzak olmali, yoksa yol tek noktaya iner
ASGARI_ILERLEME_M = 0.3


def plan_yap(izgara, res, ox, oy, robot_xy,
             sisirme_m=0.30, en_az_kume=6, kazanc=2.0, geri_cekilme_m=0.8):
    """Siradaki sinir hedefini secer ve oraya giden yolu cikarir.

    Donen (yol, bilgi). yol map cercevesinde metre cinsinden nokta listesi,
    gidilecek sinir yoksa None.
    """
    bos, dolu, bilinmiyor = maskeler(izgara)
    yaricap = int(round(sisirme_m / res))
    engel = sisir(dolu, yaricap)
    gecilebilir = bos & (~engel)

    baslangic = hucreye(robot_xy[0], robot_xy[1], res, ox, oy)
    h, w = izgara.shape
    sc, sr = baslangic

    # drone sisirilmis bolgedeyse en yakin gecilebilir hucreden basla
    if not (0 <= sr < h and 0 <= sc < w):
        return None, {"hata": "robot harita disinda"}
    if not gecilebilir[sr, sc]:
        yedek = en_yakin_gecilebilir(gecilebilir, baslangic, 20, serbest=~dolu)
        if yedek is None:
            return None, {"hata": "robotun etrafinda gecilebilir hucre yok"}
        baslangic = yedek

    mes = mesafe_alani(gecilebilir, baslangic)
    sinir = sinir_hucreleri(bos, bilinmiyor)
    kumeler = kumele(sinir, en_az=en_az_kume)

    bilgi = {"kume_sayisi": len(kumeler), "hata": None, "bitti": False}
    if not kumeler:
        bilgi["bitti"] = True
        bilgi["sebep"] = "gidilmemis sinir kalmadi"
        return None, bilgi

    en_iyi_puan = None
    en_iyi_hedef = None
    en_iyi_boy = 0
    for uyeler in kumeler:
        # kumenin en yakin ulasilabilir uyesi hedef. cok yakin uyeler atlaniyor,
        # yol kirpilinca drone hic kipirdamiyor
        asgari_hucre = (geri_cekilme_m + ASGARI_ILERLEME_M) / res
        aday = None
        aday_d = None
        for (c, r) in uyeler:
            if mes[r, c] >= asgari_hucre and (aday_d is None or mes[r, c] < aday_d):
                aday = (c, r)
                aday_d = mes[r, c]
        if aday is None:
            continue
        uzaklik_m = aday_d * res
        boy_m = len(uyeler) * res
        # yakin ve buyuk kume one cikiyor
        puan = uzaklik_m - kazanc * boy_m
        if en_iyi_puan is None or puan < en_iyi_puan:
            en_iyi_puan = puan
            en_iyi_hedef = aday
            en_iyi_boy = boy_m

    if en_iyi_hedef is None:
        # sinir var ama hicbirine gidilemiyor, bu da normal bitis
        bilgi["bitti"] = True
        bilgi["sebep"] = ("ulasilabilir sinir kalmadi (%d kume duvar ardinda)"
                          % len(kumeler))
        return None, bilgi

    yol = yol_kur(mes, gecilebilir, en_iyi_hedef)
    if yol is None:
        bilgi["hata"] = "hedef bulundu ama yol kurulamadi"
        return None, bilgi

    yol = yolu_kisalt(yol, gecilebilir)
    yol_m = [metreye(c, r, res, ox, oy) for (c, r) in yol]
    # sinirin otesi bilinmiyor, orada duvar olabilir. biraz geride duruyoruz
    yol_m = geri_cekil(yol_m, geri_cekilme_m)
    yol_m = bol(yol_m, en_fazla=1.5)

    bilgi["hedef_boy_m"] = en_iyi_boy
    bilgi["yol_uzunlugu_m"] = sum(
        math.hypot(yol_m[i + 1][0] - yol_m[i][0], yol_m[i + 1][1] - yol_m[i][1])
        for i in range(len(yol_m) - 1))
    return yol_m, bilgi


def _hat_serbest(serbest, a, b):
    """a'dan b'ye duz cizgide dolu hucre yok mu. gorus_var_mi'den farki a'ya bakmamasi."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    hata = dx - dy
    h, w = serbest.shape
    while (x0, y0) != (x1, y1):
        e2 = 2 * hata
        if e2 > -dy:
            hata -= dy
            x0 += sx
        if e2 < dx:
            hata += dx
            y0 += sy
        if not (0 <= y0 < h and 0 <= x0 < w) or not serbest[y0, x0]:
            return False
    return True


def en_yakin_gecilebilir(gecilebilir, merkez, en_fazla_hucre, serbest=None):
    """Merkezin cevresini halka halka tarayip ilk gecilebilir hucreyi bulur.

    serbest (~dolu) verilirse duvarin ote tarafindaki hucreler secilmez.
    """
    h, w = gecilebilir.shape
    c0, r0 = merkez
    for yaricap in range(1, en_fazla_hucre + 1):
        for dr in range(-yaricap, yaricap + 1):
            for dc in range(-yaricap, yaricap + 1):
                if max(abs(dr), abs(dc)) != yaricap:
                    continue
                nc, nr = c0 + dc, r0 + dr
                if not (0 <= nr < h and 0 <= nc < w and gecilebilir[nr, nc]):
                    continue
                if serbest is not None and not _hat_serbest(serbest, merkez, (nc, nr)):
                    continue
                return (nc, nr)
    return None


def ucus_noktalari(yol_m, robot_xy, esik):
    """Uculacak noktalar. Yolun ilk noktasi robotun dibindeyse atlanir.

    Robot sisirilmis bolgedeyse ilk nokta yedek hucredir ve atlanmamali,
    yolun geri kalani o hucreden kontrol edildi.
    """
    if not yol_m:
        return []
    if math.hypot(yol_m[0][0] - robot_xy[0], yol_m[0][1] - robot_xy[1]) <= esik:
        return list(yol_m[1:])
    return list(yol_m)


def cevre_engelsiz_mi(izgara, res, ox, oy, merkez_xy, yaricap_m):
    """Noktanin cevresindeki diskte dolu hucre yoksa True.

    Kalkista harita yeni oldugu icin bilinmeyen hucreler engel sayilmiyor, sadece dolu olanlar.
    """
    bos, dolu, bilinmiyor = maskeler(izgara)
    h, w = izgara.shape
    c0, r0 = hucreye(merkez_xy[0], merkez_xy[1], res, ox, oy)
    r_h = int(round(yaricap_m / res))
    for dr in range(-r_h, r_h + 1):
        for dc in range(-r_h, r_h + 1):
            if dc * dc + dr * dr > r_h * r_h:
                continue
            nc, nr = c0 + dc, r0 + dr
            if 0 <= nr < h and 0 <= nc < w and dolu[nr, nc]:
                return False
    return True


def ozet(izgara, res, ox, oy, merkez_xy=None, yaricap_m=0.6):
    """Teshis icin izgara deger dagilimi ve robot cevresinin dokumu."""
    bos, dolu, bilinmiyor = maskeler(izgara)
    satirlar = []
    satirlar.append("izgara %dx%d @ %.3f m/hucre, orijin (%.2f, %.2f)"
                    % (izgara.shape[1], izgara.shape[0], res, ox, oy))
    satirlar.append("bilinmiyor %6d | bos %6d | dolu %6d"
                    % (bilinmiyor.sum(), bos.sum(), dolu.sum()))
    satirlar.append("deger dagilimi (bilinen hucreler):")
    for alt in range(0, 100, 10):
        ust = alt + 9 if alt < 90 else 100
        n = int(((izgara >= alt) & (izgara <= ust)).sum())
        if n:
            satirlar.append("   %3d-%3d : %6d" % (alt, ust, n))
    if merkez_xy is not None:
        h, w = izgara.shape
        c0, r0 = hucreye(merkez_xy[0], merkez_xy[1], res, ox, oy)
        r_h = int(round(yaricap_m / res))
        sb = sd = sk = sdis = 0
        for dr in range(-r_h, r_h + 1):
            for dc in range(-r_h, r_h + 1):
                if dc * dc + dr * dr > r_h * r_h:
                    continue
                nc, nr = c0 + dc, r0 + dr
                if not (0 <= nr < h and 0 <= nc < w):
                    sdis += 1
                elif bilinmiyor[nr, nc]:
                    sk += 1
                elif dolu[nr, nc]:
                    sd += 1
                else:
                    sb += 1
        satirlar.append("robot map(%.2f, %.2f) -> hucre (%d, %d)"
                        % (merkez_xy[0], merkez_xy[1], c0, r0))
        satirlar.append("%.1f m disk: bos %d | bilinmiyor %d | DOLU %d | harita disi %d"
                        % (yaricap_m, sb, sk, sd, sdis))
    return "\n".join(satirlar)


def harita_yaz(temel_yol, izgara, res, ox, oy):
    """Izgarayi map_saver'in trinary bicimiyle .pgm ve .yaml olarak yazar.

    Piksel degerleri bos 254, dolu 0, bilinmiyor 205. Esikler planlayicininkiyle ayni.
    """
    piksel = np.full(izgara.shape, 205, dtype=np.uint8)
    piksel[(izgara >= 0) & (izgara <= BOS_ESIGI)] = 254
    piksel[(izgara >= DOLU_ESIGI) & (izgara <= 100)] = 0
    piksel = np.flipud(piksel)          # pgm'de ilk satir en ustte
    h, w = piksel.shape
    with open(temel_yol + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(piksel.tobytes())
    # yaml'daki esikler haritayi yuklerken kullaniliyor, trinary'de sabit
    with open(temel_yol + ".yaml", "w") as f:
        f.write("image: %s\nmode: trinary\nresolution: %.3f\n"
                "origin: [%.3f, %.3f, 0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
                % (os.path.basename(temel_yol) + ".pgm", res, ox, oy))
