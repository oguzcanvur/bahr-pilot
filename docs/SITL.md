# SITL — tekne ve sensör simülatörü

`bahr_pilot/sitl/` — otopilot yazılımının kontrol mantığını donanımsız sınamak
için. Gerçek `Vehicle`, gerçek `Estimator`, gerçek `Navigator` çalışır; yalnızca
tekne ve sensörler taklittir.

```
tekne gerçeği ─▶ SensorSuite ─▶ Estimator ─▶ Vehicle._motor_command() ─▶ motor pulse'ları ─▶ tekne
   (boat.py)       (sensors.py)  (estimator.py)        (vehicle.py)                    (rampa+gecikme+itki)
```

## Ne işe yarar, ne işe yaramaz

**Yarar:** işaretler (sağ/sol, saat yönü, kuzey/doğu), sınırlar, arıza
davranışı (GNSS kesilince ne olur, jiroskop bozulunca ne olur), kestirimci ile
navigasyonun birlikte çalışması, "geçersiz veriyle motor sürülmez" gibi
değişmezler. Hata bulduğu görüldü: bkz. `PHASE_REPORTS.md` Faz 6 ve 26–27.

**Yaramaz:** kazanç ayarı. **Tekne parametrelerinin hepsi yer tutucu**
(`boat.py: BoatParams`): kütle 25 kg, dönme ataleti 6 kg·m², itki 20 N/motor,
sürtünme katsayıları... `%60 gaz ≈ 1,5 m/s` (aracın varsayılan
`CRUISE_SPEED`/`CRUISE_THROTTLE` çifti) çıkacak şekilde seçildiler; hiçbiri
ölçülmedi. Sapma (XTE) ya da yakınsama süreleri gerçek teknenin davranışı
hakkında **hiçbir şey söylemez**. PID/Pure Pursuit kazançları gerçek gövdede
ölçülen verilerle ayarlanmalı (adım yanıtı, dönüş çemberi, durma mesafesi).

## Tekne modeli (`boat.py`)

3 serbestlik derecesi: ileri hız `u`, yana (sancağa) hız `v`, sapma hızı `r`
(saat yönü +, yani yön artar). Hızlar **suya göre**; sabit akıntı eklenir,
sürtünme suya göre hıza uygulanır.

```
m (u̇ − v r) = T_L + T_R − Xu u − Xuu |u| u + F_ileri
m (v̇ + u r) =           − Yv v − Yvv |v| v + F_yana
I  ṙ         = a (T_L − T_R) − Nr r − Nrr |r| r
```

`T_L > T_R` tekneyi **sağa** döndürür; `navigation.py` ve `motor.h` aynı kuralı
kullanır. Motor zinciri firmware'i yansıtır: pulse → ESC ölü bandı → **rampa**
(STM'nin `MOT_SLEWRATE`'i, %100/s: tam geriden tam ileriye 2 s) → ESC+pervane
birinci derece gecikme → itki (geri itki daha zayıf). RK4, iki yönlü ortalama
itkiyle **ikinci derece** (yakınsama testle doğrulanır).

Ortam: sabit akıntı (doğu, kuzey m/s) ve gövdeye sabit rüzgâr kuvveti.
Kablolama hatası: `swapped_motors=True` (sol/sağ ESC'ler yer değiştirmiş).

## Sensörler ve arıza enjeksiyonu (`sensors.py`)

Gerçek araçta kestirimcinin yediği nesneleri üretir: `ImuData` (50 Hz jiroskop,
bias + gürültü), `GnssFix` (5 Hz, gecikmeli, gürültülü), `GnssHeading` (1 Hz).
Gürültü değerleri **tahmin edicinin kendi varsayımlarından bağımsız** seçilmiştir
(filtreye tam beklediği gürültüyü vermek az şey kanıtlar) ve donanımdan
ölçülmemiştir.

Arızalar zaman pencereleridir (`Fault(kind, start_s, end_s, value)`):

| tür | etki |
|---|---|
| `gnss_outage` | konum fix'i yok |
| `heading_outage` | GNSS yönü yok |
| `imu_outage` | IMU çerçevesi hiç gelmiyor |
| `gyro_invalid` | IMU geliyor, jiroskop-geçerli biti sönük |
| `gnss_jump` | her fix `(doğu, kuzey)` m kaydırılmış |
| `heading_offset` | GNSS yönü `value`° yanlış (anten hizasızlığı) |
| `gyro_stuck` | jiroskop arıza başındaki değerde donmuş |
| `gyro_bias_step` | jiroskop bias'ı `value`°/s değişiyor |
| `gnss_degraded` | fix standalone 3D'ye düşüyor, σ 2,5 m |

## Harness (`harness.py`)

Tekne 100 Hz, kontrol ve kestirimci aracın kendi döngü hızında (20 Hz),
aradaki IMU örnekleri `Vehicle._update_estimate` gibi toplu verilir. Zaman
simüle edilir; 5 dakikalık görev ~2,5 s'de koşar. `vehicle_controller()` gerçek
`Vehicle`'ı sürer. `armed=False` motorları anında durdurur (STM gibi). Modellenmeyen:
MAVLink hattı, RC, STM failsafe'i, batarya.

## Kullanım

```python
from tests.test_vehicle import make_vehicle
from bahr_pilot.sitl.harness import Sitl, vehicle_controller
sitl = Sitl(sensors=SensorConfig(faults=(Fault("gnss_outage", 60, 80),)))
vehicle = make_vehicle()
vehicle.navigator.mission = [sitl.geodetic(0, 0), sitl.geodetic(0, 100)]
vehicle.navigator.mission_seq = 1; vehicle.state.mode = MODE_AUTO; vehicle.state.armed = True
trace = sitl.run(vehicle_controller(vehicle, sitl), 300.0)
```

Örnek senaryolar: `tests/test_sitl_vehicle.py`.

## Bilinen sınırlamalar (kayıtlı bulgular)

- **Kablolama hatası fark edilmez.** ESC uçları yer değiştirmişse tekne her
  waypoint'ten uzaklaşır ve yazılım bunu görmez (`test_swapped_motor_wiring_is_not_detected_by_the_software_today`).
  Faz 23 (kalibrasyon: komutlanan test dönüşünün IMU ile eşleşmesi) bunu ilk
  görevden önce yakalamalı.
- **Donmuş jiroskop yalnızca GNSS yönüyle ortaya çıkar** (≤1 s sonra); o süre
  içinde yön ~20° kayabilir (yer tutucu tekne, tam dönüşte). Jiroskopun kendisinden
  anlaşılamaz.
- **Anten hizasızlığı (`heading_offset`) fark edilmez**; kestirimci yönü
  doğru sanır, tekne hedefe eğri gider. Faz 20'de rota-yön tutarlılık kontrolü
  (ölçülen rota ile yön farkı) aday.
- **HOLD = nötr komut.** Görev bitince araç `HOLD`'a geçer ve sürüklenir
  (akıntıda); konum tutma yok.
