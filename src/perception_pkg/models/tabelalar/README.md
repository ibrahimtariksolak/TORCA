# Trafik tabelasi dokulari

Bu dizindeki PNG'ler `tools/tabela_uret.py` tarafindan **cizilerek**
uretilir (indirilmez). Yeniden uretmek icin:

```bash
python3 tools/tabela_uret.py            # uret + YOLO testi
python3 tools/tabela_uret.py --sadece-uret
```

## Neden cizim, indirme degil?

Elimizdeki YOLOv8 modeli GTSRB benzeri gercek tabela fotograflariyla
egitilmis. Once Gazebo'nun hazir mesh'leri denendi (sadece `stop_sign`
calisti), sonra Wikimedia Commons'tan **resmi Alman StVO tabelalarinin
gercek gorselleri** indirildi (`tools/tabela_indir.py`). Sonuc:

| tabela | gercek gorsel | elle cizim |
|---|---|---|
| Stop | 0.83 | **0.88** |
| Round-About | 0.71 | **0.87** |
| Give Way | tespit yok | **0.67** |
| Speed Limit 20/30 | `Traffic_signal` | `Traffic_signal` |
| Pedestrian Crossing | tespit yok | tespit yok |
| No Entry, donus oklari | tespit yok | tespit yok |

Resmi gorseller elle cizilenlerden **daha kotu** performans verdi.
Yani sinirlayici unsur varliklarin kalitesi degil, **modelin kendisi**:
`Speed Limit` ve `Pedestrian Crossing` siniflarini temiz bir onden
goruntuden siniflandiramiyor.

## Calisan siniflar

Kontrolcunun guvenebilecegi uc sinif var:

- `Stop_Sign`    -> dur (3 s)
- `Give Way`     -> yavasla
- `Round-About`  -> yavasla

Yaya gecidi tabeladan DEGIL, yoldaki zebra seritlerinden klasik
goruntu isleme ile algilanir; bu hem modelden bagimsiz hem daha
saglam.

Daha fazla sinif istenirse tek gercekci yol modeli Gazebo
render'lariyla ince ayara sokmaktir.
