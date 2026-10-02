# BAHR-GCS Mimarisi — Otopilot Gözünden

> **Kapsam:** Bu belge, BAHR-Autopilot'un uymak zorunda olduğu arayüzü
> tanımlamak için [bahr-gcs](https://github.com/oguzcanvur/bahr-gcs) `v0.1.0`
> kodundan **doğrudan okunarak** çıkarıldı (2026-10-02). GCS reposundaki eski
> `ARCHITECTURE.md` artık güncel değil (ör. mod yönetimini "yok" diye
> listeliyor); bu belge onun yerine geçmez, otopilot tarafının referansıdır.
> Satır numaraları `v0.1.0` içindir.

## 1. Teknoloji ve süreç modeli

| Konu | Değer |
|---|---|
| Dil | Python 3 (geliştirme ortamında 3.14) |
| Arayüz | PyQt6, `QWebEngineView` içinde Leaflet harita (`web/map.html`), Python↔JS köprüsü `QWebChannel` (`gcs/map_bridge.py`) |
| MAVLink | `pymavlink` (`common` + ArduPilot diyalekti) |
| Planlama | `shapely` + `numpy` (`gcs/mission_planner.py`) |
| Seri port / RTK | `pyserial`, kendi NTRIP istemcisi (`gcs/ntrip_client.py`) |

**Süreç modeli:** UI süreci ile MAVLink worker'ı **ayrı süreçler**
(`multiprocessing.Process`, `gcs/mavlink_service.py`). UI komutları bir
`command_queue` ile worker'a gider; worker telemetriyi ve olayları bir
`event_queue` ile geri gönderir. UI bu kuyruğu 20 Hz'de boşaltır. Worker
içinde ayrıca bir alıcı thread'i (`_recv_loop`) ve görev protokolü
mesajlarını ayıran bir `_mission_queue` vardır.

## 2. Modüller

| Dosya | Sorumluluk |
|---|---|
| `gcs/main_window.py` | Uygulama kabuğu: sol ray (LINK/VEHICLE/POWER/PILOT/CAMERA + LOG/TUNING/SETUP), harita, Plan sekmesi, uyarlanabilir hız mantığı, NTRIP kartı |
| `gcs/mavlink_worker.py` | Tüm MAVLink trafiği (bağlanma, telemetri, görev, parametre, kalibrasyon, komutlar) |
| `gcs/mavlink_service.py` | UI tarafı vekil sınıf: komutları kuyruğa koyar, olayları Qt sinyallerine çevirir |
| `gcs/mission_planner.py` | Poligon → biçerdöver (boustrophedon) tarama hattı + dönüş yayları + örnek noktaları |
| `gcs/models.py` | `TelemetryData`, `MissionPoint`, `MissionStats`, `DepthSample` veri sınıfları |
| `gcs/param_meta.py` | Parametre sözlüğü (Türkçe etiket/açıklama/aralık/birim), gruplar halinde |
| `gcs/setup_window.py` | Kurulum penceresi: genel bakış, gövde, çıkışlar, kumanda/ivmeölçer/pusula kalibrasyonu, parametre grupları, tüm parametreler |
| `gcs/ntrip_client.py`, `gcs/rtcm.py` | NTRIP istemcisi ve RTCM3 → `GPS_RTCM_DATA` parçalama |
| `sim/fake_vehicle.py` | Gazebo/SITL olmadan test için kinematik Python araç simülatörü |

## 3. Bağlantı ve araç kimliği

- Bağlantı dizesi: `udp:host:port` / `tcp:host:port` (pymavlink) veya
  `PORT,BAUD` (seri, varsayılan 115200).
- `wait_heartbeat(timeout=5)`; 5 sn'de heartbeat yoksa bağlantı başarısız.
- **Otopilot kilidi** (`_lock_onto_autopilot`): `autopilot ==
  MAV_AUTOPILOT_INVALID` olan heartbeat'ler (ADSB vb.) yok sayılır;
  `target_system`/`target_component` ilk gerçek otopilot heartbeat'ine kilitlenir.
- Bağlanınca: `REQUEST_DATA_STREAM(ALL, 10 Hz)` ve **`PARAM_REQUEST_READ("WP_RADIUS")`**
  (dönüş yayı aralığı bu değere göre hesaplanır).
- Mod adları `master.mode_mapping()` ile çözülür → heartbeat'teki
  `type=MAV_TYPE_SURFACE_BOAT`, `autopilot=ARDUPILOTMEGA` sayesinde
  **ArduRover mod numaraları** kullanılır.

## 4. GCS'nin gönderdiği MAVLink (giden)

| Eylem | Mesaj / komut | Alanlar |
|---|---|---|
| Arm / disarm | `COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM` | p1 = 1/0 |
| Mod değiştir | pymavlink `set_mode` (ArduRover mod no.) | MANUAL 0, HOLD 4, LOITER 5, AUTO 10, RTL 11, GUIDED 15 |
| Görev başlat | `MAV_CMD_MISSION_START` | — |
| Duraklat / devam | `MAV_CMD_DO_PAUSE_CONTINUE` | p1 = 0 dur, 1 devam |
| Hız değiştir | `MAV_CMD_DO_CHANGE_SPEED` | **p1 = 1 (yer hızı), p2 = hız m/s**, p3 = -1 |
| Eve dön | `MAV_CMD_NAV_RETURN_TO_LAUNCH` | — |
| Noktaya git | `COMMAND_INT MAV_CMD_DO_REPOSITION` (`MAV_FRAME_GLOBAL`) | p1 = -1, p2 = `CHANGE_MODE` bayrağı, x/y = lat/lon·1e7, z = AMSL irtifa |
| Noktaya git (yedek) | GUIDED + `SET_POSITION_TARGET_GLOBAL_INT` | yalnızca DO_REPOSITION reddedilirse |
| Ev noktası | `MAV_CMD_DO_SET_HOME` | — |
| Manuel sürüş | `RC_CHANNELS_OVERRIDE` | **CH1 = dümen, CH3 = gaz**, 1100–1900 µs (±400), diğerleri 65535; bırakınca 0 |
| RTK düzeltmesi | `GPS_RTCM_DATA` | 180 baytlık parçalar, `flags` = parça/sıra |
| Parametre | `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET` | tam liste indirme eksik indeksleri tek tek yeniden ister |
| Kalibrasyon | `MAV_CMD_PREFLIGHT_CALIBRATION` (kumanda p4, jiroskop, seviye, ivmeölçer), `MAV_CMD_ACCELCAL_VEHICLE_POS`, `MAV_CMD_DO_START/ACCEPT/CANCEL_MAG_CAL` | ArduPilot sözleşmesi |
| Motor testi | `MAV_CMD_DO_MOTOR_TEST` | p1 motor, p3 %, p4 süre |
| Yeniden başlat | `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` | — |

## 5. GCS'nin tükettiği MAVLink (gelen)

| Mesaj | GCS'de kullanımı |
|---|---|
| `HEARTBEAT` | mod adı, `SAFETY_ARMED` bayrağı, `system_status` → failsafe/sistem durumu metni |
| `GLOBAL_POSITION_INT` | konum (**lat=lon=0 → "konum yok"** sayılır), AMSL/göreli irtifa, `hdg` |
| `GPS_RAW_INT` | `fix_type`, `satellites_visible` (255 = bilinmiyor) |
| `ATTITUDE` | roll/pitch (radyan → derece) |
| `VFR_HUD` | yer hızı, yön |
| `SYS_STATUS` | batarya voltajı (65535 = yok), akım (-1 = yok), kalan % (-1 = yok) |
| `NAV_CONTROLLER_OUTPUT` | `wp_dist` |
| `MISSION_CURRENT` | aktif görev sırası → **uyarlanabilir hız buna bağlı** |
| `MISSION_ITEM_REACHED` | son öğeye ulaşılınca "görev tamamlandı" |
| `DISTANCE_SENSOR` | derinlik (cm → m) |
| `NAMED_VALUE_FLOAT` | `depth`/`sonar_depth` (m), `water_temp`/`water_temperature` (°C) |
| `RC_CHANNELS` | kumanda kalibrasyon ekranı (yalnızca kalibrasyon sırasında) |
| `PARAM_VALUE` | parametre tablosu; `WP_RADIUS` planlayıcıya da gider |
| `STATUSTEXT` | olay konsolu; kalibrasyon akışları bu metinlerle ilerler |
| `COMMAND_ACK` | komut sonucu; DO_REPOSITION reddi yedek yolu tetikler |
| `POSITION_TARGET_GLOBAL_INT` | git-noktaya hedefini yalnızca log için karşılaştırır |
| `EKF_STATUS_REPORT` | EKF durumu metni (gelmezse "UNKNOWN") |
| `MAG_CAL_PROGRESS/REPORT`, `COMMAND_LONG ACCELCAL_VEHICLE_POS` | pusula / ivmeölçer kalibrasyon ekranları |

`BATTERY_STATUS`, `HOME_POSITION`, `SERVO_OUTPUT_RAW` GCS tarafından **okunmuyor**.

## 6. Görev (mission) formatı — araca giden tam olarak budur

`upload_mission()` (`gcs/mavlink_worker.py:258`):

1. Hız verilmişse önce ayrı bir `COMMAND_LONG DO_CHANGE_SPEED(p2 = m/s)`.
2. `MISSION_CLEAR_ALL`, sonra `MISSION_COUNT = N + 1`.
3. **Slot 0 = ev yer tutucusu** (aracın o anki konumu, yoksa 0,0); tarama
   noktaları seq 1..N.
4. Her öğe: `MISSION_ITEM_INT`, `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`,
   **`MAV_CMD_NAV_WAYPOINT`**, autocontinue = 1, p2 (kabul yarıçapı) = 2.0,
   p4 = NaN, z = `MissionPoint.altitude_m`.
5. `MISSION_ACK` beklenir; ardından görev **geri okunup** seq 1..N lat/lon
   değerleri ±1 (1e-7°) toleransla karşılaştırılır.

**Görev içinde yalnızca `NAV_WAYPOINT` vardır.** `DO_CHANGE_SPEED` vb.
öğeler görevin içine konmaz.

**Araca hiç ulaşmayan bilgi:** `MissionPoint.is_turn` (dönüş noktası mı),
hat kimliği/sırası, hat aralığı, poligon, örnekleme aralığı. Bunlar yalnızca
GCS'nin belleğinde yaşar.

## 7. Tarama planlama (`gcs/mission_planner.py`)

- Poligon WGS84'ten yerel metrik düzleme çevrilir, tarama açısına göre döndürülür.
- Hatlar (lane) `line_spacing × (1 − overlap)` aralıkla süpürülür; yönleri
  sırayla tersine çevrilir (biçerdöver).
- Hat geçişlerinde, `turn_radius` > 0 ise üç yaydan oluşan bir **dönüş yayı**
  üretilir. Yay noktaları açıya göre değil mesafeye göre örneklenir
  (aralık `WP_RADIUS`'a bağlı); bu noktalar `is_turn = True` ile işaretlenir.
- `generate_sample_points()`: hat üzerinde `sample_spacing_m` aralıklı
  **beklenen** örnek noktaları. Bunlar yalnızca istatistik ve görselleştirme
  içindir, araca gönderilmez.

## 8. Uyarlanabilir hız — GCS tarafından canlı yönetilir

`main_window.py:2782–2796`: GCS `MISSION_CURRENT` değiştikçe sıradaki
hedefin `is_turn` bayrağına bakar. Dönüşteyse `survey_speed × 0.45`
(`TURN_SPEED_RATIO`), değilse `survey_speed` için yeni bir
`DO_CHANGE_SPEED` gönderir. **Sonuç:** dönüş yavaşlaması GCS–araç
bağlantısının canlı olmasına bağlıdır. Telemetri koparsa hız son ayarlanan
değerde kalır.

## 9. Parametre sistemi

- `param_meta.py`: ArduPilot adlarıyla (artı `RCMAP_ARM`, `RCMAP_OVERRIDE`,
  `BATT_FS_ENABLE` gibi bu araca özgü eklemelerle) gruplanmış sözlük:
  frame, outputs, radio, modes, navigation, tuning, battery, failsafe, sonar …
- Araçta olmayan parametre satırları, tam liste indirildikten sonra gizlenir.
  Yeni ArduPilot'ta adı değişmiş parametreler iki adla da tanımlıdır.
- `PARAM_SET`'in onayı olarak gelen `PARAM_VALUE` beklenir (`param_index =
  65535` olanlar indirme ilerlemesine sayılmaz).

## 10. Sonar / batimetri

- Derinlik kaynakları: `DISTANCE_SENSOR` (cm) veya `NAMED_VALUE_FLOAT
  depth|sonar_depth` (m).
- Her derinlik, **o anki GCS konumuyla** eşleştirilip `DepthSample` olarak
  ısı haritasına eklenir. Zaman damgası GCS'nin alım anıdır (`datetime.now()`),
  araç tarafının ölçüm zamanı değildir.
- Derinlik alarm penceresi, min/max takibi, 2D batimetri görünümü, panoya
  CSV dışa aktarma vardır. **Dosyaya otomatik kayıt yoktur.**

## 11. Korunması gereken GCS davranışları (otopilot için sözleşme)

1. Heartbeat: `MAV_TYPE_SURFACE_BOAT` + `MAV_AUTOPILOT_ARDUPILOTMEGA`, ArduRover mod numaraları.
2. `WP_RADIUS` parametresi adıyla okunabilmeli (bağlanınca istenir).
3. `DO_CHANGE_SPEED` p2 **m/s** olarak yorumlanmalı.
4. Slot 0 ev; tarama seq 1'den başlar; geri okumada lat/lon birebir dönmeli.
5. `MISSION_CURRENT` doğru akmalı (uyarlanabilir hız ve "aktif hat" buna bağlı).
6. Konum yokken `GLOBAL_POSITION_INT` lat=lon=0 gönderilmeli (GCS bunu "konum yok" sayar).
7. Manuel sürüş CH1/CH3 `RC_CHANNELS_OVERRIDE`, 0 = bırak.
8. `DO_REPOSITION` (`CHANGE_MODE` bayrağıyla) kabul edilmeli; aksi halde GCS
   yedek yola geçer.
9. Uygulanmamış komutlar `MAV_RESULT_UNSUPPORTED` dönmeli; `ACCEPTED`
   dönülürse GCS yapılmış sanar.
