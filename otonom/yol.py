"""
yol.py - bilinen bir noktaya engellerden kacarak gitme

plan_yap() hedefi kendisi seciyor, burada hedef belli. Gorev sonunda
kalkis noktasina donerken kullaniliyor. Izgara fonksiyonlari harita.py'den.
"""

from drone import DroneHatasi
import harita as H


def gecilebilir_kur(izgara, res, sisirme_m):
    """(gecilebilir, dolu) maskeleri."""
    bos, dolu, _bilinmiyor = H.maskeler(izgara)
    sisirilmis = H.sisir(dolu, int(round(sisirme_m / res)))
    return bos & ~sisirilmis, dolu


def yol_plani(izgara, res, ox, oy, baslangic_map, hedef_map,
              sisirme_m=0.44, bacak_m=1.5, hedef_kayma_hucre=12,
              baslangic_kayma_hucre=20):
    """map cercevesinde iki nokta arasinda bacaklara bolunmus yol.

    Donen (yol, not), yol kurulamazsa (None, sebep). Ucarken
    H.ucus_noktalari() ile kullan.
    """
    gecilebilir, dolu = gecilebilir_kur(izgara, res, sisirme_m)
    serbest = ~dolu

    bc, br = H.hucreye(baslangic_map[0], baslangic_map[1], res, ox, oy)
    h, w = gecilebilir.shape
    if not (0 <= br < h and 0 <= bc < w):
        return None, "baslangic harita disinda"
    if not gecilebilir[br, bc]:
        # drone sisirilmis bolgedeyse BFS oradan baslayamaz, yakindaki gecilebilir hucreye kay
        yeni = H.en_yakin_gecilebilir(gecilebilir, (bc, br),
                                      baslangic_kayma_hucre, serbest=serbest)
        if yeni is None:
            return None, "drone gecilebilir alanda degil"
        bc, br = yeni

    hc, hr = H.hucreye(hedef_map[0], hedef_map[1], res, ox, oy)
    if not (0 <= hr < h and 0 <= hc < w):
        return None, "hedef harita disinda"
    if not gecilebilir[hr, hc]:
        yeni = H.en_yakin_gecilebilir(gecilebilir, (hc, hr),
                                      hedef_kayma_hucre, serbest=serbest)
        if yeni is None:
            return None, "hedef gecilemiyor"
        hc, hr = yeni

    mes = H.mesafe_alani(gecilebilir, (bc, br))
    yol_h = H.yol_kur(mes, gecilebilir, (hc, hr))
    if yol_h is None:
        return None, "yol yok (hedef duvar ardinda)"
    yol_h = H.yolu_kisalt(yol_h, gecilebilir)
    yol_m = [H.metreye(c, r, res, ox, oy) for (c, r) in yol_h]
    yol_m = H.bol(yol_m, bacak_m)
    return yol_m, "%d bacak" % (len(yol_m) - 1)


def noktaya_uc(d, don, harita, baslangic_map, hedef_map,
               sisirme_m=0.44, irtifa=1.2, tolerans=0.25,
               zaman_asimi=60.0, bacak_m=1.5, etiket="hedef", yaz=print):
    """map cercevesindeki bir noktaya engellerden kacarak ucar.

    harita = (izgara, res, ox, oy), don kesif.py'deki Donusum.
    Varilirsa True, yol yoksa ya da hata olursa False.
    """
    izgara, res, ox, oy = harita
    yol_m, notu = yol_plani(izgara, res, ox, oy, baslangic_map, hedef_map,
                            sisirme_m=sisirme_m, bacak_m=bacak_m)
    if yol_m is None:
        yaz("%s: %s, atlandi." % (etiket, notu))
        return False

    yaz("%s -> (%.2f, %.2f) | %s"
        % (etiket, hedef_map[0], hedef_map[1], notu))
    try:
        for (mx, my) in H.ucus_noktalari(yol_m, baslangic_map, res):
            ex, ey = don.nokta(mx, my)
            d.git(ex, ey, irtifa, tolerans=tolerans, zaman_asimi=zaman_asimi)
        return True
    except DroneHatasi as e:
        yaz("%s atlandi: %s" % (etiket, e))
        return False
