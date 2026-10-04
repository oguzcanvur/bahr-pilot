# Protokoller

İki ayrı hat var ve ikisinin dili farklı. Biri standart MAVLink, diğeri bu
projeye özgü ikili **BAHR-LINK**. Ayrıntılı kaynaklar:

| Hat | Yetkili kaynak |
|---|---|
| GCS ↔ Pi (MAVLink) | `BAHR_GCS_ARCHITECTURE.md` (GCS kodundan okundu) + `bahr_pilot/vehicle.py` |
| Pi ↔ STM (BAHR-LINK) | `firmware/reflex/Core/Inc/pi_link.h` — bayt düzeni **orada** tanımlıdır; aşağıdaki tablo özet |

Her iki taraf aynı düzeni **ana makinede derlenmiş gerçek C koduyla** bayt bayt
doğrulayan testler var: `tests/test_c_firmware_logic.py`,
`tests/test_imu_link.py`, `tests/test_failsafe_c.py`.

## 1. GCS ↔ Pi: MAVLink v2, UDP 14550

BAHR-GCS bir ArduRover tekne (`MAV_TYPE_SURFACE_BOAT`,
`MAV_AUTOPILOT_ARDUPILOTMEGA`) görüyormuş gibi konuşur. Pi'nin gönderdikleri
ve anladıkları `BAHR_GCS_ARCHITECTURE.md` §4–§5'te. Bu fazlarda eklenenler:

| Ne | Nasıl |
|---|---|
| Açılışta sürüm | `STATUSTEXT` `BAHR-Pilot 0.1.1 link4 msn1` (`bahr_pilot/versions.py`) |
| Kumanda ile arm reddi | `STATUSTEXT` `PreArm: …` (gaz merkezde değil, arm anahtarı açık, IMU/batarya hazır değil) |
| Kumanda ile arm/disarm | `STATUSTEXT` `Armed (RC switch)` / `Disarmed (RC)` |
| Kumanda mod anahtarı | `MODE_CH` + `MODE1…MODE6`; MANUAL bandında GCS mod komutu `DENIED` |
| Failsafe | `STATUSTEXT` `Failsafe: <neden> (<aksiyon>)` (önem 2), `Failsafe cleared: <neden>` (önem 6), `Failsafe: mode forced to HOLD`, `Failsafe: RTL impossible, holding`, `Failsafe: disarmed`, `RTL refused: no home position` |
| Çit | `STATUSTEXT` `Fence: waypoint N is X m outside`, `Fence refused: <neden>`, `Fence stored but FENCE_ENABLE is 0`, `Failsafe: fence breached (<aksiyon>)`, `RTL refused: path to home crosses the fence` |
| Gerçek roll/pitch/hızlar | `ATTITUDE` BNO086'dan (`AHRS_ORIENTATION` uygulanmış); yaw hâlâ GNSS yönü |

Araç parametreleri (`PARAM_*`): `WP_RADIUS`, `CRUISE_SPEED`, `CRUISE_THROTTLE`,
`FS_GCS_ENABLE`, `FS_TIMEOUT`, `RCMAP_ROLL/THROTTLE/ARM`, `MODE_CH`,
`MODE1…6`, `RC1..8_MIN/MAX/TRIM/REVERSED`, `BATT_LOW_VOLT`, `BATT_FS_ENABLE`,
`AHRS_ORIENTATION`; yol takibi `NAVL1_PERIOD`, `NAVL1_DAMPING`; yön/hız kontrolü
`ATC_STR_ANG_P`, `ATC_STR_RAT_P/I/D/FF/FILT/MAX`, `ATC_SPEED_P/I/D`, `ATC_ACCEL_MAX`
(hepsi ArduRover adı; kazançlar yer tutucu tekneye göre, Faz 12); failsafe
`FS_ACTION`, `FS_EKF_ACTION`, `FS_GCS_ENABLE`, `FS_TIMEOUT`, `BATT_FS_LOW_ACT`,
`BATT_FS_CRT_ACT`, `BATT_CRT_VOLT` (Faz 20); geofence `FENCE_ENABLE`, `FENCE_TYPE`,
`FENCE_ACTION`, `FENCE_RADIUS`, `FENCE_MARGIN` ve araca özgü `FENCE_COAST_DECEL` (Faz 21).

**Görev protokolü `mission_type`'a göre ayrılır (Faz 21):** 0 görev (slot 0 = ev; `CLEAR_ALL`
yanıtsız, BAHR-GCS buna bağlı), 1 geofence (`MISSION_COUNT/ITEM_INT/REQUEST_*/CLEAR_ALL`;
öğeler `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION/EXCLUSION` 5001/5002 ve
`..._CIRCLE_INCLUSION/EXCLUSION` 5003/5004, param1 = köşe sayısı ya da yarıçap; geçersizse
`MAV_MISSION_INVALID`, sıra bozuksa `INVALID_SEQUENCE`), 2 rally (`MAV_MISSION_UNSUPPORTED`).
Çit RAM'de tutulur (Faz 22).
**Derinlik (Faz 17-19):** `DISTANCE_SENSOR` kabul edilen her sondaj için **bir kez**, süzülmüş ve
`RNGFND1_OFFSET` eklenmiş değerle (cm), `min/max_distance` = `RNGFND1_MIN/MAX`; reddedilen (tepe, menzil
dışı) ve ısınma okumaları gönderilmez. Eskiden her telemetri turunda ham değer 5 Hz tekrarlanıyordu
(BAHR-GCS her mesajda haritaya nokta ekler). Kalite sınıflı batimetri örnekleri (VALID / LOW_QUALITY / INVALID)
Pi'de tutulur ve görev günlüğüne (`<log-dir>/missions/mission-…/bathymetry.csv`, Faz 25) yazılır, GCS'ye gitmez.

**Sağlık tablosu (Faz 24, `diagnostics.py`):** `SYS_STATUS.onboard_control_sensors_present / enabled / health`
artık gerçek (eskiden 0, 0, 0) ve `load` ana döngünün doluluğudur (binde, son 5 s). Bit başına:

| bit | var | etkin | sağlıklı |
|---|---|---|---|
| `3D_GYRO` | STM varsa | evet | IMU OK |
| `3D_ACCEL` | STM varsa | **hayır** (kullanılmıyor) | IMU OK |
| `GPS` | evet | evet | konum tahmini OK (ölü hesap ya da yok → sağlıksız) |
| `LASER_POSITION` (ekolot) | `--echomap-port` varsa | evet | veri taze ve güvenilir |
| `RC_RECEIVER` | STM varsa | evet | RC sinyali var |
| `MOTOR_OUTPUTS` | STM varsa | evet | STM telemetrisi taze |
| `BATTERY` | STM varsa | evet | voltaj okunuyor, düşük değil |
| `AHRS` | evet | evet | yön geçerli |
| `XY_POSITION_CONTROL` | evet | yalnızca otonom modda | konum **ve** yön OK |
| `LOGGING` | `--log-dir` varsa | evet | günlük hatasız |
| `GEOFENCE` | evet | `FENCE_ENABLE` | ihlal yok |

`health` yalnızca *var olan* ve OK olan parçalar için işaretlenir; UYARI da "sağlıksız" sayılır (veri eksik
ya da bozuk). `enabled & ~health` yer istasyonunda kırmızı çıkar. **BAHR-GCS bu alanları okumaz**
(`_system_status_name` her zaman "OK" der); QGroundControl ve Mission Planner gösterir. `HEARTBEAT.system_status`
hâlâ her zaman `MAV_STATE_ACTIVE`: BAHR-GCS onu "failsafe" satırında gösterdiği için değiştirilmedi.
Her seviye değişimi bir `STATUSTEXT` olur ve görev günlüğünün olay dosyasına düşer: `Parça: neden`
(`GNSS: no position`, `battery: 10.0 V low`, `STM link: restarted (1)`, `sonar: no data for 6 s`), önem 4 (uyarı),
3 (hata), düzelince 6 (`GNSS: recovered`). Açılıştan sonraki ilk 10 s'de bir şey duyurulmaz, aynı parça en çok
5 s'de bir konuşur. Failsafe nedenleri tabloda görünür ama ikinci kez duyurulmaz (`Failsafe: ...` zaten söyler).

**Doğrulama ve kalıcılık (Faz 22):** her `PARAM_SET` aralık/tamsayı/küme denetiminden geçer;
reddedilen değer için eski değer `PARAM_VALUE` ile geri yankılanır ve `STATUSTEXT`
`Param <ad> rejected: <neden>` gelir. Kabul edilenler `--param-file` dosyasına (varsayılan
`~/.bahr_pilot/params.json`, yalnızca varsayılandan farklılar, atomik) yazılır.
`MAV_CMD_PREFLIGHT_STORAGE` param1: 0 yeniden yükle, 1 kaydet, 2 varsayılana sıfırla.

Konum mesajları kestirimden gelir (Faz 6): `GLOBAL_POSITION_INT` süzülmüş konum +
hız (vx kuzey, vy doğu, cm/s) + yön; `GPS_RAW_INT` alıcının ham değeri (rota alanı
gerçek rota); `NAV_CONTROLLER_OUTPUT` `nav_bearing` = yol takipçisinin hedef yönü,
`xtrack_error` = çizgiden metre (sağ +).

## 2. Pi ↔ STM: BAHR-LINK, UART 115200 8N1

Hepsi **büyük endian**, her çerçevenin sonunda önceki tüm baytların XOR'u.

| Yön | Çerçeve | Sync | Uzunluk | Hız | İçerik |
|---|---|---|---|---|---|
| Pi → STM | Motor komutu | `A5 5A` | 7 | 20 Hz | motor 1 (sol), motor 2 (sağ) darbe µs |
| Pi → STM | RC ayarı | `C5 5C` | 25 | değişince | kanal haritası, kalibrasyon, mod anahtarı kanalı + MANUAL bant maskesi |
| STM → Pi | Telemetri | `E5 5E` | 44 | 10 Hz | 16 ham SBUS kanalı, durum bitleri, batarya mV, roll/pitch, mod bandı, arm durumu + engel nedeni |
| STM → Pi | **IMU** | `E6 6E` | 28 | 50 Hz | µs zaman damgası, quaternion Q14, jiroskop Q9 (rad/s), lineer ivme Q8 (m/s²), geçerlilik bitleri |

**Zaman damgası:** IMU çerçevesindeki `t_us` STM'nin µs saatidir (32 bit,
71.6 dakikada bir sarar). Pi bunu `timesync.StmClock` ile kendi monoton
saatine eşler: HSI osilatörünün ±%1 sürüklenmesini ve UART gecikmesini
1 saniyelik kovalardaki en düşük ofsetlere (son 60 kova) doğru bir çizgi
uydurarak ayıklar; geç gelen çerçeveler (negatif olamayan gecikme) tahmini bozmaz.

**Gönderme tarafı:** STM çerçeveleri 256 baytlık bir halka tamponuna koyar
(tamamen sığmıyorsa çerçeve **bütün olarak** atılır, yarım gitmez). Normal
trafik STM → Pi yönünde 50 Hz × 28 B + 10 Hz × 44 B = 1.84 KB/s, hattın
11.5 KB/s (115200 baud, 8N1) kapasitesinin ~%16'sı.

**Alıcı taraf (Pi):** `extract_frames()` bozuk veri ve sahte sync'te yalnızca
**bir bayt** atar, arkasındaki gerçek çerçeveyi yutmaz.

### Sürümler

`bahr_pilot/versions.py` içindeki `BAHR_LINK_VERSION`: çerçeve düzeni değişince
artırılır (1: ilk, 2: mod anahtarı, 3: arm durumu, **4: IMU çerçevesi**).
STM ve Pi **birlikte** güncellenmelidir; sürüm uyuşmazlığını bugün çalışma
zamanında tespit eden bir el sıkışma yok (bilinen eksik).

## 3. MAVLink'e geçiş (Faz 13 kararı)

Prompt Pi↔STM hattında da standart MAVLink ister. BAHR-LINK şimdilik çalışan,
host'ta iki yönlü doğrulanmış bir protokol; STM'yi küçük tutuyor. MAVLink'e
geçiş, host'ta `pymavlink` ile bayt bayt doğrulanabildiği için düşük riskli
ama kazancı (araç uyumluluğu, TIMESYNC, PARAM) STM'nin parametre/görev
taşımaya başlamasına bağlı. Faz 13'te yeniden karar verilecek.
