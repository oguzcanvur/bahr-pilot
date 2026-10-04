# Changelog

Bu proje [Semantic Versioning](https://semver.org/) kullanır: `MAJOR.MINOR.PATCH`.

- **PATCH** (`0.0.x`) — hata düzeltmeleri, davranış değişikliği yok
- **MINOR** (`0.x.0`) — geriye uyumlu yeni özellik
- **MAJOR** (`x.0.0`) — geriye uyumsuz değişiklik (1.0.0'a kadar bu proje için "kararlılık" anlamına gelmez, sadece kapsamlı değişiklik anlamına gelir)

Biçim [Keep a Changelog](https://keepachangelog.com/) temel alınarak tutulur.

## [Unreleased]

## [0.2.0] — 2026-10-04

Hepsi yazılım, protokol, derleme ve simülasyon düzeyinde doğrulandı; gerçek donanımda
(GNSS, ekolot, Nucleo, BNO086, ESC'ler, tekne) denenmedi. Tüm kazançlar, gürültüler, gecikmeler
ve eşikler yer tutucu tekne ve tahmin değerlerdir. Faz başına ölçümler: `docs/PHASE_REPORTS.md`.

**Uyumluluk:** Pi ↔ STM protokolü BAHR-LINK 4'e çıktı (RC config 25 bayt, telemetri 44 bayt, yeni IMU
çerçevesi). Pi yazılımı ile Nucleo firmware'i **birlikte** güncellenmeli; v0.1.1 firmware'i ile
çalışmaz. Yeni parametrelerin BAHR-GCS'de Türkçe açıklamalı ve aralık denetimli görünmesi için BAHR-GCS
0.2.0 önerilir.

### Eklenen (Pi tarafı, `bahr_pilot/`)
- **Jeodezi** (`geo.py`, Faz 10): tam WGS84, yerel teğet düzlem (`LocalFrame`, tam ters dönüşüm),
  kesin yön/mesafe/çapraz iz hatası. Eski küresel haversine kilometrede 1,3–2,6 m hata veriyordu.
- **Durum kestirimi** (`estimator.py`, Faz 6): yön filtresi (yön + jiroskop sapması, 4σ kapısı,
  geçerlilik kuralları), gecikme telafili konum filtresi, en çok 5 s ölü hesap, `Pose`
  (konum, yön, hız, σ, roll/pitch). Navigasyon yalnızca `state.pose`'u kullanır, ham alanları değil.
- **Hat izleme** (`pathfollow.py`, Faz 11): ArduRover'ın `NAVL1_PERIOD/DAMPING` parametreleriyle
  ileri bakışlı pure pursuit; çapraz iz kazancı, varış kuralı.
- **Yön ve hız kontrolü** (`control.py`, Faz 12): ArduRover yapısı ve adları (`ATC_STR_*`, `ATC_SPEED_*`,
  `ATC_ACCEL_MAX`); ölçüm üzerinden türev, anti-windup, kötü dt işleme; yer tutucu teknelerin en
  kötüsüne göre ayarlı.
- **Failsafe** (`failsafe.py`, Faz 20): GCS kaybı, batarya düşük/kritik, konum kaybı, yön kaybı, çit ihlali;
  eylemler REPORT/RTL/HOLD/TERMINATE; bekleme süreleri, histerezis, seviye tetiklemeli (neden sürerken
  yeniden AUTO seçmek yine HOLD'a götürür).
- **Coğrafi çit ve güvenli RTL** (`geofence.py`, Faz 21): ev çevresinde daire + yüklenen çokgen/daire
  dahil/hariç bölgeler (MAVLink çit öğeleri), işaretli marj, öngörülü ihlal denetimi (duruş mesafesi),
  çiti kesecek RTL reddedilir.
- **Parametre doğrulama ve kalıcılık** (`params.py`, Faz 22): her parametre için aralık/tamsayı/küme
  tablosu, reddedilen değerin eski değerle yankılanması, atomik JSON kalıcılığı (`--param-file`),
  bozuk dosya `.corrupt` olarak kenara alınır, `MAV_CMD_PREFLIGHT_STORAGE` (yükle/kaydet/sıfırla),
  SIGTERM'de son kayıt.
- **Ekolot süzgeci** (`sonar.py`, Faz 17): menzil, tepe/basamak ayrımı (Theil-Sen eğilim + hızlı
  öngörücü), gürültü kalitesi, ısınma işareti. **Batimetri** (`bathymetry.py`, Faz 18–19): kat edilen
  mesafeye göre örnekleme, yankı konumunun yaş + gecikme kadar geri alınması, VALID / LOW_QUALITY /
  INVALID sınıflaması (nedenleriyle).
- **Görev günlüğü** (`missionlog.py`, Faz 25): `--log-dir` ile her silahlanma için
  `missions/mission-<zaman>/` (`meta.json`, `bathymetry.csv`, `track.csv`, `events.csv`, `summary.json`);
  yazma hatası kaydı durdurur ama döngüyü asla düşürmez.
- **Sağlık tablosu** (`diagnostics.py`, Faz 24): 11 parça için OK/UYARI/HATA, debounce, açılış süresi,
  parça başına hız sınırı; her seviye değişimi `STATUSTEXT` + olay günlüğü; `SYS_STATUS`
  `present/enabled/health` bitleri ve ana döngü yükü (`load`); açılıştaki tablo `meta.json`'da.
- **Simülatör** (`bahr_pilot/sitl/`, Faz 26–27, `docs/SITL.md`): 3 serbestlik dereceli tekne, sensör
  modeli ve 10'dan fazla arıza türü; gerçek `Vehicle` döngüsü içinde çalışır.
- Dokümanlar: `docs/PHASE_REPORTS.md`, `docs/SITL.md`, `docs/SENSOR_INTERFACE.md`; `ARCHITECTURE.md` durum
  sütunu; `MAVLINK_PROTOCOL.md` (parametreler, failsafe/çit mesajları, görev tipleri, derinlik, sağlık bitleri).

### Eklenen (diğer)
- **Kumandada ArduPilot tarzı mod anahtarı** (`MODE_CH` + `MODE1…MODE6`,
  BAHR-GCS'nin zaten tanıdığı parametreler). Anahtarın kanalı 6 PWM
  aralığına bölünür; her aralığın modunu `MODEn` söyler. Varsayılan, 3
  konumlu bir anahtar için: MANUAL / HOLD / AUTO.
  - `MODEn = MANUAL` olan aralıkta **STM motorları kumandadan kendisi sürer**
    (Pi/GCS olmadan, eskiden `RCMAP_OVERRIDE` ile aynı bağımsızlıkta).
  - Diğer modları Pi uygular: anahtar başka aralığa geçince (ve açılışta bir
    kez) o aralığın modu seçilir; aradan GCS de mod değiştirebilir.
  - Anahtar MANUAL'dayken GCS başka moda geçemez; komut `DENIED` döner.
  - Sinyal kaybında mod korunur, motorları STM zaten durdurur.
- `firmware/reflex/Core/Src/mode_switch.c`: saf (HAL'siz) aralık mantığı.
- `tests/test_c_firmware_logic.py`: gerçek firmware C kodunu (`mode_switch.c`
  ve `pi_link.c`, HAL taklit edilerek) host derleyicisiyle derleyip gerçek
  Python tarafıyla bayt bayt karşılaştırıyor (config 25 bayt, telemetri 43
  bayt, motor çerçevesi, flash yazma debounce'u). Derleyici yoksa atlanır.

### Düzeltilen
- `LocalFrame.to_geodetic` `to_enu`'nun tam tersi değildi (artık tek bir kalıntı düzeltmesiyle tam).
- Navigasyon küresel haversine kullanıyordu; darbe uzunlukları `int()` ile kırpılıyordu (`round()`).
- Eve dönüş noktası yalnızca ilk çağrıda ayarlanıyordu.
- Tahminci gecikme için belirsizliği şişiriyordu ve RTK doğruluğunu öldürüyordu (artık ölçüm ileri
  taşınıyor, gecikmenin belirsizliği gürültüye ekleniyor).
- Yön, jiroskop arızasında 180°'ye kadar yanlışken "geçerli" kalıyordu.
- GCS yaşı duvar saatiyle ölçülüyordu; araç tek bir monotonik saat kullanıyor. **STM telemetri tazeliği de
  aynı hatayı taşıyordu** (`time.time()` ile damgalı): Pi'de RTC yok, NTP/GNSS saat senkronu STM'yi bir an
  ölü gösterebilirdi. Artık `time.monotonic()`.
- `mission_type` yok sayılıyordu: çit yüklemek görevi ezebilirdi.
- `FS_TIMEOUT=NaN` GCS failsafe'ini kapatabiliyordu (artık parametre doğrulaması reddeder).
- Açılışta Pi'nin varsayılan RC haritası STM'nin flash'taki kalibrasyonunun üzerine yazılıyordu (artık
  yalnızca Pi'de kaydedilmiş STM'ye ait değer varsa gönderilir).
- `DISTANCE_SENSOR` her telemetri turunda (5 Hz) ham değeri tekrarlıyordu; BAHR-GCS her mesajda haritaya
  nokta eklediğinden aynı sondaj 5 kopya çiziliyordu. Artık kabul edilen her sondaj için bir kez, süzülmüş.
- Görev günlüğünde her görevin ilk batimetri örneği düşüyordu (adım sırası); özet mesafe durağan
  konum titreşimini yol sayıyordu (102,0 m yerine 104,7 m).

### Değişen
- `SYS_STATUS` sensör bit alanları ve `load` artık gerçek (eskiden 0). BAHR-GCS bunları okumaz; QGC/Mission
  Planner gösterir.
- `NAV_CONTROLLER_OUTPUT` gerçek `nav_bearing` ve `xtrack_error` taşıyor.
- Görev protokolü `mission_type`'a göre ayrıldı (0 görev, 1 çit, 2 toplanma → UNSUPPORTED). Tip 0
  `MISSION_CLEAR_ALL` bilerek ONAYLANMAZ: BAHR-GCS her `MISSION_ACK`'ı yükleme döngüsünde sayıyor.
- `RcTelemetry.last_update` artık monotonik saniye.
- `RCMAP_OVERRIDE` kaldırıldı, yerini `MODE_CH` aldı. Pi↔STM protokolü
  değişti: RC config çerçevesi 20 → 25 bayt, telemetri 42 → 43 bayt
  (`mode_slot`). İki taraf birlikte güncellenmeli. Flash kayıt düzeni
  değişti (24 → 32 bayt, yeni sihirli sayı); eski kayıtlar reddedilip
  varsayılanlara dönülür.

## [0.1.1] — 2026-10-02

### Düzeltilen
- **Klonlanan repo çalışmıyordu:** Python modülleri repo kökündeydi, bu
  yüzden `python -m bahr_pilot.vehicle`, `pytest` ve systemd servisi
  `ModuleNotFoundError` veriyordu. Modüller `bahr_pilot/` alt paketine
  taşındı. Artık repo kökünden çalıştırılıyor.
- `MAV_CMD_DO_CHANGE_SPEED` hızı yüzde sanıyordu (GCS'nin 1.5 m/s'si %5
  gaza kırpılıyordu). Bu, hem görev yüklemede hem de GCS'nin her dönüşte
  gönderdiği uyarlanabilir hız komutunda motoru neredeyse durduruyordu.
  Artık m/s olarak yorumlanıyor.
- BAHR-GCS bağlanınca `WP_RADIUS` istiyor ama araç `WP_RADIUS_M` diyordu;
  navigator parametreyi hiç okumadan sabit 3.0 kullanıyordu. İkisi de düzeltildi.
- Mod butonuyla AUTO'ya geçilince araç ev noktasında (seq 0) bekliyordu;
  artık 1. noktadan başlıyor.
- Uygulanmamış reboot komutu `ACCEPTED` dönüyordu; artık `UNSUPPORTED`.
- Firmware: RC harita ayarı UART kesmesinin içinde flash'a yazılıyordu.
  Artık ana döngüde, yalnızca disarm iken ve ardışık yazımlar 1 sn içinde
  birleştirilerek yazılıyor.
- Firmware: debugger breakpoint'inde watchdog MCU'yu resetliyordu.

### Değişen
- Parametre adları BAHR-GCS'nin Türkçe parametre sözlüğündeki ArduPilot
  adlarına geçti: `CRUISE_FRAC` → `CRUISE_SPEED` + `CRUISE_THROTTLE`,
  `WP_RADIUS_M` → `WP_RADIUS`, `GCS_FS_TIMEOUT_S` → `FS_TIMEOUT` +
  `FS_GCS_ENABLE`.

### Eklenen
- `docs/BAHR_GCS_ARCHITECTURE.md`: BAHR-GCS'nin araca gönderdiği ve
  beklediği her şey, koddan okunarak çıkarıldı.
- `docs/ARCHITECTURE_REVIEW.md`: hedef mimariye (STM32/FreeRTOS + ROS 2)
  göre boşluk analizi, uyumluluk matrisi ve karar listesi.

## [0.1.0] — 2026-10-01

### Eklenen
- Nucleo (`firmware/reflex/`) tarafı: ESC PWM çıkışı, SBUS RC alıcı okuma,
  Pi ile `pi_link` paket protokolü (çift yönlü — motor komutu + RC harita
  ayarı + telemetri), donanımsal arm switch + öncelik tablolu failsafe,
  BAHR-GCS'nin `RCMAP_*`/`RCn_MIN/MAX/TRIM/REVERSED` parametreleriyle
  yapılandırılabilir throttle+steering karıştırma.
- Pi (`bahr_pilot/`) tarafı: BAHR-GCS ile tam MAVLink uyumlu araç süreci —
  telemetri, mod/arm/görev protokolü, RTD100+echoMAP sensör okuyucuları,
  RTCM düzeltme iletimi, waypoint navigasyonu.
- LED yakma donanımda doğrulandı (gerçek NUCLEO-G431RB kartında).
- BNO086 IMU sürücüsü (`firmware/reflex/Core/Src/imu.c`, SHTP/I2C, Game
  Rotation Vector raporu) — roll/pitch artık `ATTITUDE` mesajında gerçek
  (yaw hâlâ GNSS yönünden). Protokol sabitleri bilinen çalışan bir açık
  kaynak sürücüden alındı, ama bu modül hiç gerçek BNO086'ya karşı
  çalıştırılmadı.
- Batarya voltaj okuma (`firmware/reflex/Core/Src/battery.c`, ADC1) ve
  Pi tarafında gerçek `SYS_STATUS.voltage_battery` + düşük voltaj
  failsafe'i (`BATT_LOW_VOLT`/`BATT_FS_ENABLE` parametreleri,
  varsayılan kapalı). ADC gerilim bölücü oranı ölçülmedi, yer tutucu.
- Donanımsal bekçi sayacı (IWDG, doğrudan register erişimiyle, 500 ms).
- RC harita/kalibrasyon ayarları artık flash'a kalıcı yazılıyor
  (`firmware/reflex/Core/Src/settings.c`) — güç kesilince kaybolmuyor.
- Ham sensör verisi kaydı (`bahr_pilot/datalog.py`, NDJSON,
  `--log-dir` ile açılır).
- `bahr_pilot/tests/` — donanımsız pytest paketi (gerçek `NucleoLink`,
  `RtcmReassembler`, `Vehicle` sınıflarını loopback seri port ve doğrudan
  çağrılarla test ediyor).
- `bahr_pilot/deploy/` — Pi için systemd servis şablonu ve SSH üzerinden
  güncelleme betiği (henüz gerçek Pi'de denenmedi).
- Proje kendi başına bir GitHub reposu oldu:
  [bahr-pilot](https://github.com/oguzcanvur/bahr-pilot).

### Bilinen eksikler
- Gerçek RTD100/echoMAP/Nucleo/BNO086/batarya donanımıyla uçtan uca hiç
  test edilmedi — tüm doğrulama bu oturumda MAVLink/paket protokolü,
  derleme ve `pytest` seviyesinde yapıldı.
- Batarya ADC gerilim bölücü oranı kalibre edilmedi — raporlanan voltaj
  kaba bir tahmin, multimetreyle doğrulanmadan düşük voltaj failsafe'i
  açılmamalı (zaten varsayılan kapalı).
- BNO086'nın tekneye montaj yönüne göre roll/pitch işareti/ekseni
  doğrulanmadı.

[Unreleased]: https://github.com/oguzcanvur/bahr-pilot/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/oguzcanvur/bahr-pilot/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/oguzcanvur/bahr-pilot/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/oguzcanvur/bahr-pilot/releases/tag/v0.1.0
