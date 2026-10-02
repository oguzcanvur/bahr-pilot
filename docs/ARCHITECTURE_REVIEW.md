# ARCHITECTURE REVIEW — BAHR-Autopilot (Faz 0)

**Tarih:** 2026-10-02 · **İncelenen:** bahr-pilot `v0.1.0` (+ bu fazdaki
düzeltmeler), bahr-gcs `v0.1.0` · **Referans:** "BAHR AUTOPILOT — MASTER
DEVELOPMENT PROMPT"

## Yöntem — neye dayanıyor

- BAHR-GCS'nin MAVLink, görev, planlama, parametre ve batimetri kodu
  **okundu**. Sonuç ayrı belgede: [`BAHR_GCS_ARCHITECTURE.md`](BAHR_GCS_ARCHITECTURE.md).
- **GCS'nin gerçek MAVLink worker sınıfı** (`gcs/mavlink_worker.py::_Worker`,
  yeniden yazılmış bir taklit değil) gerçek bir `bahr_pilot.vehicle`
  sürecine UDP üzerinden bağlanarak uçtan uca çalıştırıldı: bağlanma,
  `WP_RADIUS` okuma, `set_speed`, görev yükleme + geri okuma doğrulaması,
  AUTO'ya geçiş, arm.
- Yayınlanmış `v0.1.0` **sıfırdan klonlanıp** çalıştırılmaya çalışıldı.
- Firmware `make clean && make` ile derlendi; Python tarafı `pytest`
  (30 test) ile koşuldu.
- **Gerçek donanımın hiçbiri bu fazda kullanılmadı.** Aşağıda "doğrulandı"
  ifadesi yazılım/protokol seviyesini kasteder.

---

## 0. DÜZELTME — rol dağılımı (2026-10-02, kullanıcı itirazı)

Bu belgenin ilk sürümü prompt'taki "STM32 = IMU + GNSS + EKF + navigasyon +
yol takibi + PID" dağılımını hedef aldı. **Bu, projenin başından beri
kararlaştırılmış mimariyle çelişiyordu** ve çelişki kullanıcıya gösterilmeden
benimsendi. Geçerli dağılım (11 Eylül'deki orijinal karar, kullanıcı tarafından
yeniden doğrulandı):

| Katman | Sorumluluk |
|---|---|
| **STM32 (refleks, hafif)** | Kumandayı (SBUS) okumak, **manuel sürüş** (kumanda → karıştırıcı → motor, Pi'siz), motor çıkışı (karıştırıcı, sınırlar, rampa, acil durdurma), **BNO086'yı okumak ve Pi'ye iletmek**, batarya ölçümü, arm kapıları, failsafe, watchdog, kendi ayarlarının (RC haritası) kalıcılığı |
| **Raspberry Pi (beyin)** | **GNSS**, durum kestirimi (konum/yön füzyonu), navigasyon, hat takibi, yön/hız kontrolü (Pi motor komutu gönderir), görev/tarama yönetimi, batimetri, sonar, kayıt, tanılama, MAVLink köprüsü (GCS'ye) |
| **BAHR-GCS** | Planlama, izleme, parametre, manuel (joystick) sürüş, görselleştirme |

**Bunun faz planına etkisi:**

- **STM32'de EKF, GNSS, waypoint navigasyonu, Pure Pursuit, yön/hız PID'si,
  görev durum makinesi YOK.** Prompt'taki Faz 5–6 ve 10–12 (GNSS, EKF, nav,
  hat takibi, PID) ve 15–16 (görev, tarama) **Pi/ROS 2 tarafında**
  yapılır. Faz 4 (IMU) STM tarafında "oku ve ilet"e iner.
- **FreeRTOS gerekmiyor.** Mevcut kesme tabanlı süper-döngü bu iş yükü için
  yeterli; görev planı (§6.6) ve MCU RAM tartışması (§6.4, D4) boşa düştü.
  STM büyürse (ör. kalıcı parametre tablosu, çok sensör) o zaman yeniden
  bakılır.
- **D1/D2 sonuçları** (aşağıda) artık "STM32 EKF'si GNSS'i Pi'den alır"
  diye okunmamalı: STM'de EKF yok. Pi kaybında STM'nin yapacağı şey zaten
  bugünkü gibi **dur** (RTL STM'de değil, Pi'dedir).
- **Hâlâ geçerli:** §2'deki hata düzeltmeleri, §3–§4 (GCS uyumluluğu ve
  tarama görevi analizi), §6.2–§6.3, §6.5, §6.7–§6.10 (§6.5 yalnızca UART
  sayımı için), §8 (ROS 2 topic'leri Pi tarafında), §9.
- **Geçersiz kalan:** §6.1'in A/B+ ayrımı (görevi STM tutmuyor), §6.4, §6.6,
  §5'teki tabloda "STM32" varsayan satırlar (Faz 3, 5, 6, 9–12'nin STM'ye
  ait kısımları).

**Prompt'taki iki madde bu düzende zaten karşılanıyor:** Faz 7 (RC +
manuel mod: kumanda STM'ye bağlı, Pi olmadan sürer) ve §13/§19'daki
`/cmd_heading`, `/cmd_speed` (Pi'nin ürettiği istenen yön/hız). İkincisi
ileride STM'ye yön/hız hedefi gönderme seçeneğini açık bırakıyor; bugün
Pi motor komutunu doğrudan gönderiyor ve bir teknenin saniyelik yön
dinamiği için Pi'nin ~20 Hz'lik döngüsü yeterli görünüyor (ölçülmedi).

---

## 1. Özet

**Bugünkü durum:** bahr_pilot iki parçadan oluşuyor. Pi'de tek süreçli bir
Python MAVLink aracı çalışıyor (ROS 2 yok). STM32'de FreeRTOS'suz,
bare-metal bir "refleks" firmware'i var. İkisi kendi tasarımımız olan ikili
bir UART protokolüyle (`pi_link`) konuşuyor. GNSS ve sonar Pi'ye bağlı;
navigasyon da Pi'de.

**Hedef (prompt):** STM32/FreeRTOS sensörleri, EKF'yi, kontrolü ve
güvenliği Pi'den bağımsız yürütecek. Pi/ROS 2 görev, batimetri, kayıt ve
tanılamayı üstlenecek. İki hop'ta da MAVLink kullanılacak.

**En büyük boşluklar** (önem sırasına göre):

1. **Hat takibi yok.** Navigasyon "noktanın kerterizine dön" mantığında;
   çapraz hat hatası (cross-track error) kontrolü yok. Akıntı ve rüzgarda
   tarama hatları kayar, bu da doğrudan batimetri kalitesini bozar (Faz 10–12).
2. **Durum kestirimi yok.** EKF yok, IMU yalnızca BNO086'nın kendi füzyon
   çıktısı (roll/pitch), zaman senkronizasyonu yok (Faz 3–6).
3. **Tarama yapısı araca ulaşmıyor.** GCS hat kimliğini ve dönüş bilgisini
   göndermiyor. Prompt'taki tarama durum makinesi ve hat kimlikli batimetri
   örnekleri bu haliyle imkânsız (Faz 15–16, GCS değişikliği gerektirir).
4. **Mimari katman ayrımı henüz yok.** ROS 2 yok, FreeRTOS yok,
   Pi↔STM32 MAVLink değil; Pi tarafı tek bir 650 satırlık dosya (`vehicle.py`).
5. **Parametreler kalıcı değil** (Pi tarafı). GCS'den yapılan her ayar
   Pi yeniden başlayınca kayboluyor. Yalnızca RC haritası STM32 flash'ında.
6. **Güvenlik/teşhis eksik.** Yapılandırılabilir failsafe aksiyonları yok,
   geofence yok, heartbeat'te failsafe durumu bildirilmiyor, MAVLink imzası yok.

---

## 2. Bu fazda bulunan ve düzeltilen hatalar

Prompt'un "önce analiz" kuralına uyuldu. Yeni özellik yazılmadı; yalnızca
analiz sırasında **doğrulanan**, güvenliği ya da GCS uyumluluğunu bozan
hatalar düzeltildi.

| # | Önem | Hata | Kök neden | Düzeltme | Doğrulama |
|---|---|---|---|---|---|
| 1 | **Kritik** | Yayınlanan `v0.1.0` klonlanınca hiç çalışmıyordu: README komutu, testler ve systemd servisi `ModuleNotFoundError: bahr_pilot` veriyordu | Repo ayrılırken Python dosyaları repo **köküne** kondu; paket adı klasör adından gelir, klon klasörü `bahr-pilot` | Modüller `bahr_pilot/` alt paketine taşındı (`git mv`, geçmiş korundu) | Sıfırdan klonda hata **yeniden üretildi**; düzeltme sonrası repo kökünden `python -m bahr_pilot.vehicle` ve `pytest` çalışıyor |
| 2 | **Yüksek** | `DO_CHANGE_SPEED` hızı yüzde sanıyordu: GCS'nin 1.5 m/s'si `1.5/100` → %5 tabana kırpılıyordu | GCS p2'yi m/s gönderiyor (`mavlink_worker.py:461`) | Hedef hız m/s olarak tutuluyor; gaz = `CRUISE_THROTTLE × hedef / CRUISE_SPEED` (ArduRover ileri besleme sözleşmesi) | Birim testi + gerçek GCS worker'ı: araç logunda `target speed -> 1.20 m/s` |
| 2a | — | Etkisi: hızlı her görev yüklemesi **ve** GCS'nin her dönüşte gönderdiği uyarlanabilir hız komutu motoru neredeyse durduruyordu | — | (2 ile çözüldü) | — |
| 3 | Orta | GCS bağlanınca `WP_RADIUS` istiyor, araç `WP_RADIUS_M` diyordu; navigator parametreyi hiç okumayıp sabit 3.0 kullanıyordu | İsim uyuşmazlığı + parametre navigator'a bağlanmamış | Parametre `WP_RADIUS` oldu ve `Navigator.step()`'e geçiriliyor | Birim testi + gerçek GCS worker'ı bağlanırken `WP_RADIUS = 3.0` aldı |
| 4 | Orta | Araca özgü parametre adları (`CRUISE_FRAC`, `WP_RADIUS_M`, `GCS_FS_TIMEOUT_S`) GCS'nin Türkçe parametre sözlüğünde yoktu | Tasarım | ArduPilot adları: `WP_RADIUS`, `CRUISE_SPEED`, `CRUISE_THROTTLE`, `FS_TIMEOUT`, `FS_GCS_ENABLE` (davranış korunarak, GCS-kaybı failsafe'i varsayılan açık) | Birim testleri |
| 5 | Orta | Firmware: RC harita ayarı UART **kesmesinin içinde** flash'a yazılıyordu | `DecodeConfigFrame` ISR'dan çağrılıyor, oradan `Settings_SaveRcMap` | Kesme yalnızca çerçeveyi saklıyor. Ana döngü uyguluyor ve flash'a yalnızca **disarm iken** ve son çerçeveden **1 sn sonra** yazıyor (RadioPage'in onlarca `PARAM_SET`'i tek silmeye iniyor) | Temiz derleme. **Donanımda test edilmedi.** |
| 5a | — | Etkisi: tek banklı G431'de sayfa silme işlemi on milisaniyeler boyunca komut okumayı durdurur. Kesme içinde bu süre boyunca SBUS ve Pi UART baytları taşabilir (overrun), bu yüzden kalibrasyon yazımlarının bir kısmı kaybolabilirdi. Ayrıca kontrol döngüsü yarım güncellenmiş bir haritayı okuyabiliyordu | — | `s_rcMap` artık yalnızca ana döngüde yazılıyor | — |
| 6 | Düşük | Mod butonuyla AUTO'ya geçilince araç seq 0'da (ev) bekleyip hiçbir şey yapmıyordu | `MISSION_START` dışında seq 1'e geçilmiyordu | AUTO'ya girişte seq < 1 ise 1'den başlanıyor (ArduRover davranışı) | Birim testi + gerçek GCS worker'ı |
| 7 | Düşük | Uygulanmamış reboot komutuna `ACCEPTED` dönülüyordu; GCS yeniden başlatıldı sanıyordu | — | `MAV_RESULT_UNSUPPORTED` | Birim testi |
| 8 | Düşük | Debugger'da breakpoint'te durunca IWDG MCU'yu resetliyordu | IWDG debug sırasında dondurulmamış | `DBGMCU_APB1FZR1_DBG_IWDG_STOP` | Temiz derleme |

---

## 3. BAHR-GCS ↔ bahr_pilot uyumluluk matrisi

✓e = gerçek GCS worker'ıyla uçtan uca doğrulandı · ✓ = birim testi veya kod
okuması · ⚠ = kısmi · ✗ = yok

### GCS → araç

| GCS eylemi | Durum | Not |
|---|---|---|
| Bağlan / otopilot kilidi | ✓e | `SURFACE_BOAT` + `ARDUPILOTMEGA` heartbeat |
| `WP_RADIUS` okuma | ✓e | bu fazda düzeltildi |
| Parametre listesi / okuma / yazma | ✓ | yazımlar **kalıcı değil** (Pi yeniden başlayınca kaybolur) |
| Mod: MANUAL / HOLD / AUTO / GUIDED / RTL | ✓e (AUTO), ✓ | |
| Mod: **LOITER** | ⚠ | numarası tanınıyor ama konum tutma yok; HOLD gibi motor kesiyor, tekne sürüklenir |
| Arm / disarm | ✓e | AUTO/GUIDED/RTL'de GPS fix şartı, MANUAL'da yok |
| Görev yükleme + geri okuma | ✓e | |
| `MISSION_START`, `DO_PAUSE_CONTINUE` | ✓ | |
| `DO_CHANGE_SPEED` | ✓e | bu fazda düzeltildi (m/s) |
| RTL | ⚠ | eve düz çizgide gidiş; geofence/engel yok, varınca HOLD |
| `DO_REPOSITION` (git-noktaya) | ✓ | `CHANGE_MODE` bayrağıyla GUIDED'a geçer |
| `DO_SET_HOME` | ✓ | |
| `RC_CHANNELS_OVERRIDE` (CH1/CH3) | ✓ | MANUAL modda |
| `GPS_RTCM_DATA` | ✓ | RTD100'e yazılıyor (Pi USB) |
| Kumanda kalibrasyonu | ✓ | RadioPage sözleşmesi |
| İvmeölçer / jiroskop / seviye / pusula kalibrasyonu | ✗ | `UNSUPPORTED` dönüyor |
| Motor testi | ✓ | |
| Reboot | ✗ | artık dürüstçe `UNSUPPORTED` |

### Araç → GCS

| Mesaj | Durum | Not |
|---|---|---|
| `HEARTBEAT` | ⚠ | `system_status` hep `ACTIVE`; failsafe durumunda `CRITICAL/EMERGENCY` bildirilmiyor |
| `GLOBAL_POSITION_INT`, `GPS_RAW_INT`, `VFR_HUD` | ✓e | konum yokken 0,0 (GCS "konum yok" sayar) |
| `ATTITUDE` | ✓ | roll/pitch BNO086'dan (geçerliyse), yaw GNSS'ten |
| `SYS_STATUS` | ⚠ | voltaj var; akım ve kalan % yok; sensör sağlık bitleri hep 0 |
| `NAV_CONTROLLER_OUTPUT`, `MISSION_CURRENT`, `MISSION_ITEM_REACHED` | ✓e | |
| `DISTANCE_SENSOR`, `NAMED_VALUE_FLOAT water_temp` | ✓ | echoMAP'ten |
| `RC_CHANNELS` | ✓ | Nucleo telemetrisinden |
| `EKF_STATUS_REPORT` | ✗ | GCS "UNKNOWN" gösteriyor |
| `BATTERY_STATUS`, `HOME_POSITION`, `SERVO_OUTPUT_RAW`, `AUTOPILOT_VERSION` | ✗ | prompt istiyor; GCS şu an okumuyor |

---

## 4. Tarama görevi: GCS'den araca ne gidiyor, ne kayboluyor

Araca giden: slot 0 ev + N × `NAV_WAYPOINT` + ayrı bir `DO_CHANGE_SPEED`.
Dönüş yavaşlaması (`× 0.45`) **GCS'den canlı** geliyor, çünkü GCS
`MISSION_CURRENT`'ı izleyip her geçişte yeni bir `DO_CHANGE_SPEED` gönderiyor.

Kaybolan: `is_turn`, hat kimliği, hat sınırları, hat aralığı, poligon,
örnekleme aralığı.

**Sonuçlar:**

- Prompt'taki `GO_TO_START → LINE_CAPTURE → SURVEY_LINE → TURN → NEXT_LINE`
  durum makinesi, hat kimlikli batimetri örneği (`survey_line_id`) ve hat
  bazlı çapraz hat hatası, araç bu bilgiyi almadıkça mümkün değil.
- Uzun menzilde telemetri koparsa dönüş yavaşlaması da durur.

**Öneri (Faz 15–16, geriye uyumlu):** Araç bir `MISSION_FORMAT_VERSION`
(veya `AUTOPILOT_VERSION` içinde BAHR imzası) yayınlar. GCS bunu görürse
genişletilmiş görev yükler: `NAV_WAYPOINT`'lerin arasına hat başı/sonu ve
dönüş işaretçisi olarak BAHR diyalektinde özel bir `MAV_CMD`
(`line_id`, tür, hedef hız) koyar. Görmezse bugünkü düz listeyi gönderir.
Böylece GCS gerçek ArduPilot/SITL ile de çalışmaya devam eder (ArduPilot
bilinmeyen komutu reddederdi). Bu bir **GCS değişikliği** gerektirir.

---

## 5. Boşluk analizi — faz bazında

| Faz | Konu | Bugün | Boşluk | Donanım şart mı |
|---|---|---|---|---|
| 1 | Mimari + arayüzler | Bu belge | `MAVLINK_PROTOCOL.md`, `SENSOR_INTERFACE.md`, mesaj/topic tanımları | Hayır |
| 2 | Donanım soyutlama | Her sürücünün kendi API'si (`Battery_*`, `IMU_*`, `SBUS_*`) | Ortak arayüz (`init/update/getStatus/getTimestamp`); ölçüm yapısı (`timestamp, value, quality, valid, source`) | Hayır (doğrulama evet) |
| 3 | Zaman | Yalnızca `HAL_GetTick()` (ms) | µs monoton saat, GNSS PPS yakalama, Pi↔STM32 `TIMESYNC` | Kısmen |
| 4 | IMU + attitude | BNO086 Game Rotation Vector → roll/pitch | Ham ivme/jiroskop/manyetometre raporları (EKF için), kalibrasyon, kalıcılık | **Evet** |
| 5 | GNSS | Pi'de: BESTPOSA/HEADINGA + NMEA; fix tipi eşleme var | STM32'ye taşıma (karar D1); BESTPOSA'nın std. sapma ve `diff_age` alanları (NovAtel düzenine göre mevcut) okunmuyor; kalite sınıflandırma ve geçerlilik kontrolü | **Evet** |
| 6 | EKF / INS | Yok | 15 durumlu ES-EKF, inovasyon kapısı, ölü hesap zaman aşımı | Hayır (sim), evet (ayar) |
| 7 | RC + manuel | Gaz/dümen/arm/elle-devralma ✓ (STM32) | Mod ve acil durdurma kanalları | Doğrulama evet |
| 8 | Motor | Karıştırıcı + 1000–2000 µs kırpma | Ölü bant, rampa/ivme sınırı, yön ters çevirme, motor bazında min/max | Evet |
| 9 | Arm / güvenlik | İki bağımsız kapı (Pi armed + donanım anahtarı) | `BOOT→…→ARMED` durum makinesi, ön-arm kontrolleri (IMU/EKF/batarya/RC) | Kısmen |
| 10 | Waypoint nav | Pi'de haversine/kerteriz | Merkezi koordinat kütüphanesi (WGS84/ECEF/ENU/NED/BODY), XTE/ATE | Hayır |
| 11 | Hat takibi | **Yok** | Pure Pursuit + algoritma arayüzü | Hayır (sim) |
| 12 | Yön + hız PID | Açık çevrim P (yön), ileri besleme (hız) | Yön ve hız PID'leri, `ATC_*` parametreleri | Ayar evet |
| 13 | ROS 2 köprüsü | Yok | `bahr_mavlink` düğümü | Karar D3 |
| 14 | ROS 2 navigasyon | Yok | Hedef yol/yön/hız üretimi | — |
| 15 | Görev durum makinesi | Yalnızca ArduRover modları | entry/update/exit durum makinesi | Hayır |
| 16 | Tarama | Düz waypoint listesi | Hat yapısı (bkz. §4), hat yakalama, araç tarafı dönüş hızı | **GCS değişikliği** |
| 17 | Sonar | echoMAP NMEA (SDDPT/SDDBT, 1 Hz) ayrıştırma | Aralık kontrolü, aykırı değer, medyan, kalite, ofset/draft | Evet |
| 18 | Batimetri örnekleme | Zaman tabanlı (5 Hz) NDJSON log | Mesafe tabanlı örnek, örnek yapısı | Kısmen |
| 19 | Batimetri QC | Yok | VALID/LOW_QUALITY/INVALID | Hayır |
| 20 | Failsafe | GCS kaybı → dur, batarya → dur (varsayılan kapalı); STM32: RC kaybı, Pi kaybı, arm anahtarı, watchdog | Aksiyonlar yapılandırılamıyor (IGNORE/WARN/HOLD/STOP/RTL/ABORT); GNSS/EKF/sonar/IMU kaybı izlenmiyor | Kısmen |
| 21 | Geofence + RTL | RTL düz çizgi | Poligon geofence, güvenli RTL | Hayır |
| 22 | Parametreler | Pi: bellekte dict (**kalıcı değil**); STM32: yalnızca RC haritası flash'ta | Tip/min/max/kalıcılık; sahibine (STM32/Pi) göre yönlendirme | Hayır |
| 23 | Kalibrasyon | Yalnızca kumanda | İvme/jiroskop/manyetometre/sonar/motor | Evet |
| 24 | Tanılama | Yok | Sağlık tablosu, olay seviyeleri | Hayır |
| 25 | Kayıt | Tek NDJSON dosyası | Görev klasörü + CSV'ler + olay/failsafe logları | Hayır |
| 26–28 | Sim / SITL / HIL | GCS'nin kinematik `sim/fake_vehicle.py`'si; bahr_pilot'ta sim yok | Dinamik model, gürültü ve arıza enjeksiyonu, HIL | Hayır / evet |

---

## 6. Mimari kararlar ve öneriler

### 6.1 Prompt'un iç tutarsızlığı: görev ve navigasyon nerede yaşar?

Prompt aynı anda görev ve tarama yönetimini **Pi'ye** veriyor (§2, §4,
Faz 14–16), `firmware/stm32/` ağacına ise `mission/state_machine`,
`mission/survey`, `navigation/pure_pursuit` koyuyor (§3). Faz 14 de
"ROS 2 yol/yön/hız üretsin, STM32 kontrol etsin" diyor. Bu çelişkinin
çözülmesi gerekiyor:

- **Seçenek A — ArduPilot modeli.** Görevi STM32 tutar ve AUTO'yu baştan
  sona STM32 uçurur. Pi bir "companion" olur: batimetri, kayıt, teşhis,
  MAVLink yönlendirme.
  - Artısı: Pi çökse bile tarama sürer.
  - Eksisi: görev protokolü, parametreler, tarama mantığının tamamı C'de
    ve 32 KB RAM'li bir MCU'da yazılır.
- **Seçenek B+ — prompt'un literal okuması (önerilen).** Görev ve tarama
  sıralamasını Pi/ROS 2 yapar (hat sırası, örnekleme, QC, durum makinesi).
  STM32 ise o anki hedef segmenti (başlangıç, bitiş, hız, `line_id`)
  ve 1–2 segmentlik ön belleği izler: Pure Pursuit, PID, karıştırıcı,
  güvenlik. Ev konumunu tutar; Pi kaybında `PI_HEARTBEAT_TIMEOUT` sonrası
  yapılandırılmış aksiyonu (HOLD/RTL) **kendi başına** uygular.
  - Neden önerilen: prompt §30 ile birebir uyumlu ("AUTO → HOLD/RTL/
    yapılandırılmış aksiyon"). GCS'nin gözünde "otopilot" bugün olduğu gibi
    Pi'deki köprü düğümü olur, yani **GCS değişikliği gerekmez**. C tarafı
    da G431'e sığacak kadar küçük kalır.

### 6.2 GNSS STM32'ye mi, Pi'ye mi?

Prompt EKF'yi STM32'ye koyuyor, dolayısıyla GNSS ölçümü de orada olmalı.
Bugün RTD100 Pi'ye USB ile bağlı.

RTD100'ün **hem USB-C hem 3.3 V TTL UART'ı** var (veri sayfası). Bu da bir
hibrit çözümü mümkün kılıyor: TTL UART'tan STM32'ye konum ve yön logları,
USB'den Pi'nin RTCM enjeksiyonu. Bu hibrit, alıcının bir portta log
verirken diğerinden RTCM kabul ettiğine dayanıyor. Bu, Bynav tabanlı
alıcılarda tipik bir davranış ama **bu cihazda doğrulanmadı**. Karar D1.

### 6.3 Sonar nerede?

Prompt sonarı STM32 sürücüleri arasında sayıyor. echoMAP NMEA 0183'ü
**1 Hz** veriyor ve derinlik bir kontrol döngüsü girdisi değil, bir
batimetri büyüklüğü.

**Öneri:** Sonar Pi'de (batimetri katmanında) kalsın. STM32'ye yalnızca
"minimum derinlik / karaya oturma" güvenlik sinyali gitsin (istenirse).
echoMAP STM32'ye bağlanacaksa NMEA 0183 seviye dönüştürücü gerekir.

Ayrıca: 1 Hz ve bilinmeyen gecikme, 1.5 m/s'de hat boyunca ~1.5 m'ye
kadar konum–derinlik kaymasına yol açar. Faz 3'te gecikme karakterize
edilmeli.

### 6.4 MCU kaynakları (G431RB: 128 KB flash, 32 KB RAM)

- **Bugün:** ~40.4 KB flash / ~3.0 KB RAM (`-O0`, FreeRTOS'suz).
- **Eklenecek:** FreeRTOS, MAVLink (yalnızca kullanılan mesajlar,
  header-only), 15 durumlu ES-EKF (15×15 float kovaryans ≈ 0.9 KB +
  çalışma matrisleri), navigasyon, parametre tablosu, ~6–8 görev yığını.
- **Tahmin** (ölçülmedi): flash'a sığar; RAM **sıkışık** olur.
- **Alternatif:** NUCLEO-G474RE (512 KB / 128 KB, aynı G4 ailesi ve HAL,
  aynı Nucleo-64 formu) düşük riskli bir yükseltme.
- **Bu karar Faz 2'den önce verilmeli** (D4), çünkü CubeMX projesi
  FreeRTOS'la bir kez yeniden üretilecek.

### 6.5 Donanım çevre birimi bütçesi (G431, cihaz header'ından doğrulandı)

- **UART:** USART1 (SBUS), USART3 (Pi), LPUART1 (ST-LINK VCP) dolu.
  **USART2 ve UART4 boş** (UART5 yok). GNSS ve sonar ikisi de STM32'ye
  gelirse ikisini de tüketir. Pin çakışmaları CubeMX'te kontrol edilmeli;
  ör. USART2'nin alternatif pinleri SWD ve PA15 (motor 1) ile çakışabilir.
- **Zamanlayıcı:** G431'de **TIM5 yok**. Tek 32-bit zamanlayıcı TIM2, o da
  ESC PWM'inde. µs zaman damgası için `DWT->CYCCNT` (170 MHz, yazılımla
  64-bit'e genişletilmiş) önerilir. GNSS PPS'i (RTD100'de var) boş bir
  zamanlayıcının input-capture kanalına bağlanmalı.

### 6.6 FreeRTOS görev planı (taslak, Faz 2'de kesinleşecek)

| Görev | Öncelik | Hız | Not |
|---|---|---|---|
| Safety/Failsafe | en yüksek | 100 Hz | bugünkü `Failsafe_Update` |
| Control (PID + karıştırıcı) | yüksek | 50–100 Hz | |
| Estimation | yüksek | IMU hızında tahmin, GNSS'te düzeltme | |
| Sensors (IMU I2C, GNSS UART DMA) | orta | | SBUS ISR'da kalır |
| Comms (MAVLink Pi linki) | orta | 50 Hz | |
| Params/Storage | en düşük | olay tabanlı | flash yalnızca disarm'da (bu fazdaki düzeltmenin genellemesi) |
| Diagnostics | düşük | 1 Hz | |

Watchdog boşta döngüden değil, **her görevin kalp atışını kontrol eden**
bir denetleyiciden beslenmeli.

### 6.7 Koordinat çerçeveleri

- **lat/lon float32'de tutulmamalı.** Türkiye enlemlerinde (32–64 aralığı)
  float32 çözünürlüğü 2⁻¹⁸ ° ≈ **0.42 m**, boylamda (16–32) ≈ 0.16 m. RTK'nın
  cm seviyesini yok eder. Saklama: int32 (1e-7 °) veya double. Kontrol:
  ev/görev başlangıcı orijinli yerel **ENU float32** (10 km'de bile mm
  çözünürlük).
- ROS 2 tarafı **ENU** (REP 103). MAVLink/havacılık tarafı **NED**.
  Dönüşüm yalnızca sınırda ve tek bir kütüphanede yapılmalı.
- **Bugünkü bir tutarsızlık:** RTD100 `HEADINGA` **gerçek** kuzeye göre;
  echoMAP yedeği `HCHDM` **manyetik**. Sapma düzeltmesi yapılmadan
  karıştırılıyor (Türkiye'de ~5–6°). Yedeğe düşüldüğünde navigasyon
  sistematik olarak sapar. Düzeltilmeli ya da yedek reddedilmeli.
- Prompt kuralı "duran teknede COG'yi yön kabul etme": bugünkü kod COG
  kullanmıyor ✓.

### 6.8 Pi↔STM32 protokolü

`pi_link` yerine MAVLink v2 (`c_library_v2`, header-only; `common` +
BAHR diyalekti) önerilir.

- **Artıları:** standart araçlar (mavproxy, Wireshark dissector),
  `TIMESYNC`, `PARAM_*` protokolü bedavaya gelir.
- **Hız:** 115200 baud dar kalır; 921600'e çıkılmalı.
- **İmza:** MAVLink imzası radyo hattında (GCS↔Pi) zorunlu; kablolu iç
  hatta isteğe bağlı.

### 6.9 ROS 2 platformu

- Pi şu an **Raspberry Pi OS (Debian 13, arm64)** çalıştırıyor.
- ROS 2 Jazzy'nin (LTS, 2029'a kadar) resmi binary desteği (Tier 1)
  **Ubuntu 24.04 amd64/arm64**. Debian yalnızca Tier 3 ve yalnızca amd64.
- Resmi doküman Raspberry Pi OS için **Docker içinde Ubuntu** öneriyor.
- **Seçenekler:** (a) Pi'yi Ubuntu Server 24.04'e yeniden kurmak (temiz ve
  destekli; mevcut SSH/ağ/servis kurulumu yeniden yapılır), (b) mevcut
  OS'ta Docker `ros:jazzy` (seri port geçişi çalışır, bir katman ekler),
  (c) kaynaktan derleme (önerilmez). Karar D3.

### 6.10 Güvenlik

- **ESC tipi doğrulanmalı.** Açılışta 1500 µs nötr gönderiliyor. Bu
  yalnızca **çift yönlü** ESC'lerde "dur" demek. Tek yönlü bir ESC'de
  1500 µs **yarım gaz** demektir. Pervaneler takılıyken ilk enerji
  verilmeden önce kontrol edilmeli (D5).
- **UDP 14550 açık.** MikroTik köprüsündeki herhangi bir cihaz tekneye
  komut verebilir. MAVLink imzası ve ağ izolasyonu Faz 20/29'da şart.

---

## 7. Hedef repo yapısı — mevcut dosyaların yeri

Prompt'un §3 düzeni hedef alınıyor. Bu fazda yalnızca Python paketi alt
klasöre taşındı (hata #1). Geri kalan taşımalar ilgili fazda yapılacak.

| Bugün | Hedef | Ne zaman |
|---|---|---|
| `firmware/reflex/` (CubeIDE, bare-metal) | `firmware/stm32/` (FreeRTOS, `App/`, `drivers/`, `estimation/` …) | Faz 2 — CubeMX projesi FreeRTOS ile yeniden üretilirken; şimdi taşımak IDE workspace'ini bir kez daha bozar |
| `bahr_pilot/` (tek süreç Python) | `ros2_ws/src/bahr_mavlink`, `bahr_vehicle`, `bahr_navigation`, `bahr_logging` … | Faz 13–14. O zamana kadar GCS'ye karşı çalışan tek Pi tarafı bu, korunuyor |
| `tests/` | `tests/` (+ host'ta derlenen C birim testleri) | Faz 2'den itibaren |
| `deploy/` | `configs/` + `tools/` | Faz 25–29 |
| — | `docs/`, `simulation/`, `configs/` | ilgili fazlarda |

---

## 8. ROS 2 paket / topic / QoS taslağı

Mümkün olan yerde standart mesaj tipleri, gerekirse `bahr_msgs`.

| Topic | Tip | QoS | Gerekçe |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | BEST_EFFORT, sensor data | yüksek hız; eski ölçümün yeniden gönderimi anlamsız |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | BEST_EFFORT, sensor data | aynı |
| `/gnss/status` | `bahr_msgs/GnssStatus` (RTK durumu, `diff_age`, HDOP, std. sapma) | RELIABLE | durum geçişleri kaçırılmamalı |
| `/gnss/heading` | `bahr_msgs/GnssHeading` (çift anten, geçerlilik) | BEST_EFFORT | |
| `/sonar/depth` | `bahr_msgs/SonarDepth` (ham, filtreli, kalite, geçerli, zaman) | BEST_EFFORT | |
| `/ekf/state` | `nav_msgs/Odometry` (ENU) | BEST_EFFORT | |
| `/vehicle/state`, `/vehicle/mode`, `/vehicle/health` | `bahr_msgs/*` | RELIABLE | komut/durum |
| `/cmd_heading`, `/cmd_speed`, `/navigation/target` | `bahr_msgs/*` | RELIABLE | kayıp komut tehlikeli; doğrulama STM32'de |
| `/mission/state`, `/mission/current_line` | `bahr_msgs/*` | RELIABLE + TRANSIENT_LOCAL | geç katılan düğüm son durumu görmeli |
| `/bathymetry/sample` | `bahr_msgs/BathymetrySample` | RELIABLE | her örnek kayda girmeli |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RELIABLE | ROS standardı |

---

## 9. Parametre sistemi ilkeleri (Faz 22)

- **Ad:** anlamı aynıysa ArduPilot adı (GCS'nin Türkçe sözlüğü ve
  ArduPilot alışkanlığı bedavaya gelir), yoksa `BAHR_` öneki.
- **Her parametre:** ad, tip, varsayılan, min, max, birim, kalıcı mı,
  **sahibi** (STM32 / Pi).
- **Tek tablo:** GCS tek bir tablo görür. Pi köprüsü `PARAM_SET`'i sahibine
  yönlendirir. STM32 parametreleri flash'ta (genelleştirilmiş `settings.c`,
  disarm'da yazılır), Pi parametreleri dosyada.
- **Bugünkü parametreler** (bu fazdan sonra): `WP_RADIUS`, `CRUISE_SPEED`,
  `CRUISE_THROTTLE`, `FS_GCS_ENABLE`, `FS_TIMEOUT`, `RCMAP_*`,
  `RC1..8_MIN/MAX/TRIM/REVERSED`, `BATT_LOW_VOLT`, `BATT_FS_ENABLE`.

---

## 10. Test stratejisi

| Seviye | Bugün | Hedef |
|---|---|---|
| Python birim | 30 `pytest` testi (`tests/`) | her ROS 2 paketi için |
| C birim (host) | **yok** — yalnızca `arm-none-eabi` derlemesi | saf mantık modülleri (karıştırıcı, failsafe öncelikleri, SBUS çözme, settings serileştirme, EKF matematiği) host'ta derlenip test edilmeli; bu makinede host C derleyicisi yok, Pi'deki `gcc` veya MSYS2/WSL kullanılabilir |
| Entegrasyon (GCS↔araç) | GCS'nin gerçek worker'ı ↔ gerçek araç süreci, bu fazda elle koşuldu | iki repoyu birlikte gerektirdiği için GCS reposunda kalıcı bir entegrasyon testine dönüştürülmeli |
| SITL | GCS'nin kinematik simülatörü | dinamik model + gürültü + arıza enjeksiyonu (GNSS/RC/GCS/Pi kaybı, batarya, geofence) |
| HIL | yok | ROS 2 sim → MAVLink → gerçek STM32, sanal aktüatör |
| Saha | yok | Faz 30 |

---

## 11. Kullanıcı kararı gereken konular

| # | Karar | Seçenekler | Öneri |
|---|---|---|---|
| **D1** | GNSS nereye bağlanacak | STM32 TTL UART / Pi USB / hibrit | **Hibrit** (önce RTD100'ün iki portu aynı anda kullanılabiliyor mu doğrulanmalı) |
| **D2** | Görevin sahibi | A: STM32 (ArduPilot modeli) / B+: Pi sıralar, STM32 segment izler + kendi HOLD/RTL'si | **B+** |
| **D3** | ROS 2 platformu | Ubuntu 24.04 yeniden kurulum / mevcut OS'ta Docker / kaynaktan | Ubuntu 24.04 ya da Docker |
| **D4** | MCU | G431RB'de kal / NUCLEO-G474RE | Faz 2'den önce bütçe çıkarılıp karar verilmeli; RAM sıkışırsa G474RE |
| **D5** | ESC tipi | çift yönlü mü? | **İlk enerji öncesi zorunlu kontrol** |
| **D6** | Sonar | echoMAP (1 Hz NMEA) / ayrı bir yankı iskandili | Faz 17'de değerlendirilmeli |

---

### Verilen kararlar (2026-10-02) — §0 düzeltmesiyle birlikte oku

| # | Karar | Sonuç |
|---|---|---|
| D1 | **GNSS Pi'de kalıyor** (USB) | Orijinal mimariyle aynı. GNSS, EKF ve navigasyon Pi'de. **Pi kaybında** STM'nin kendi başına yapabileceği şey **dur** (bugünkü failsafe tablosu); RTL zaten Pi'nin işi. |
| D2 | ~~B+~~ **Geçersiz** (§0) | Görevi ve segment izlemeyi STM'ye vermek, "STM hafif" kararıyla çelişiyor. Görev, tarama, hat takibi ve kontrol Pi'de; STM motoru ve kumandayı sürer, IMU'yu iletir. GCS'nin gözünde otopilot yine Pi'deki köprü, **GCS değişikliği gerekmez**. |
| D3 | **Docker'da ROS 2 Jazzy** (mevcut Raspberry Pi OS üstünde) | Pi'nin mevcut SSH/ağ/servis kurulumu korunur. Konteyner seri portlara (`/dev/serial/by-id/*`, `/dev/serial0`) ve ağa (MAVLink UDP) erişecek şekilde çalıştırılacak. |
| D4 | ~~G431RB RAM bütçesi~~ **Boşa düştü** (§0) | STM'de EKF ve FreeRTOS olmadığı için G431RB bol bol yetiyor (~40 KB / 128 KB flash, ~3 KB / 32 KB RAM). |
| D5 | ESC tipi | Açık. İlk enerji öncesi zorunlu kontrol. |
| D6 | Sonar | Açık. Faz 17'de. |

## 12. Faz 1'in gereksinimleri

- D1–D4 verildi (yukarıda). Mimari belgesi bu kararlarla yazılacak.
- Çıktılar: `docs/ARCHITECTURE.md` (karar verilmiş mimari),
  `docs/MAVLINK_PROTOCOL.md` (GCS↔Pi ve Pi↔STM32 mesaj seti, BAHR
  diyalekti, ID aralıkları), `docs/SENSOR_INTERFACE.md` (ortak sürücü
  arayüzü + ölçüm yapısı), `bahr_msgs` taslağı.
- Kod: yalnızca arayüz başlıkları ve tipler; davranış değişikliği yok.
