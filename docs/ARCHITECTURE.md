# BAHR-Autopilot Mimarisi

> **Durum:** Faz 1 çıktısı (2026-10-02). Karar kayıtları ve gerekçeler için
> [`ARCHITECTURE_REVIEW.md`](ARCHITECTURE_REVIEW.md), BAHR-GCS'nin araçtan
> beklediği her şey için [`BAHR_GCS_ARCHITECTURE.md`](BAHR_GCS_ARCHITECTURE.md).

## 1. Üç katman, bir ilke

```
                    BAHR-GCS  (PC, PyQt6)
                        │
                        │  MAVLink v2 / UDP 14550  (kablolu Ethernet veya MikroTik köprüsü)
                        ▼
        ┌────────────────────────────────────────┐
        │  Raspberry Pi 4  —  "beyin"            │
        │  bahr_pilot (Python)  →  ileride ROS 2 │
        │                                        │
        │  GNSS · durum kestirimi · navigasyon   │
        │  hat takibi · yön/hız kontrolü         │
        │  görev · batimetri · kayıt · tanılama  │
        └───────┬────────────────────────┬───────┘
                │ UART 115200            │ USB
                │ BAHR-LINK              ├── RTD100 GNSS (Bynav, çift anten)
                ▼                        └── Garmin echoMAP 42cv (NMEA 0183)
        ┌───────────────────────┐
        │  STM32 NUCLEO-G431RB  │
        │  "refleks" — hafif    │
        │                       │
        │  kumanda · motor      │
        │  IMU okuma · batarya  │
        │  arm/failsafe · WDG   │
        └──┬───────┬───────┬────┘
           │SBUS   │I2C    │PWM
        RC alıcı  BNO086   2 × ESC (diferansiyel itki)
```

**İlke: STM küçük ve basit kalır.** Güvenliği ve kumandayı Pi'den bağımsız
tutar; zekâ Pi'dedir. Bu, projenin başından beri (2026-09-11) kararlaştırılan
dağılımdır ve 2026-10-02'de kullanıcı tarafından yeniden doğrulanmıştır
(bkz. `ARCHITECTURE_REVIEW.md` §0). Prompt'taki "STM32 = EKF + navigasyon +
PID + görev" dağılımı **kullanılmıyor**.

## 2. Sorumluluklar

| Katman | Yapar | Yapmaz |
|---|---|---|
| **STM32** | SBUS kumanda okuma; **manuel sürüş** (kumanda → karıştırıcı → ESC, Pi'siz); motor çıkışı (karıştırma, sınırlar, rampa, acil durdurma); arm durum makinesi ve ön-arm kontrolleri; BNO086'yı okuyup zaman damgalı Pi'ye iletme; batarya ölçümü; failsafe öncelik tablosu; donanım watchdog'u; kendi ayarlarının flash'ta kalıcılığı | GNSS, EKF/konum, navigasyon, hat takibi, görev, sonar, kayıt, ağ, dosya sistemi |
| **Raspberry Pi** | GNSS ve sonar okuma; konum/yön kestirimi; navigasyon ve hat takibi; yön ve hız kontrolü (motor komutu üretir); görev/tarama yürütme; batimetri örnekleme ve kalite kontrol; kayıt; tanılama; GCS ile MAVLink; failsafe aksiyonları (HOLD/RTL/…); geofence; parametre kalıcılığı | Motor güvenliğinin son sözü (bu STM'de) |
| **BAHR-GCS** | Planlama, izleme, parametre, joystick ile manuel sürüş, görselleştirme | — |

## 3. Güvenlik katmanları (motoru kim durdurabilir)

Üsttekiler alttakileri ezer; **ilk üçü Pi ve GCS ölü olsa bile çalışır.**

1. **Donanım:** kumandadaki fiziksel arm anahtarı kapalı → motor nötr.
2. **STM watchdog (IWDG):** firmware kilitlenirse MCU resetlenir, açılış durumu "dur".
3. **STM failsafe tablosu:** RC kaybı → dur; Pi 500 ms sessiz → dur (MANUAL bandı hariç); açılışta/arm olmadan → dur.
4. **Pi failsafe'leri (yapılandırılabilir aksiyon):** GCS kaybı, batarya, GNSS/EKF kaybı, geofence.
5. **GCS:** operatör komutu.

Kumandada mod anahtarı MANUAL bandındayken STM sürüşü kumandadan yapar ve
GCS başka moda geçemez (komut `DENIED`): kumanda her zaman son sözü söyler.

## 4. Ara yüzler

| Hat | Protokol | Belge |
|---|---|---|
| GCS ↔ Pi | MAVLink v2, UDP 14550; ArduRover diyalekti (`MAV_TYPE_SURFACE_BOAT` + `ARDUPILOTMEGA`) | `BAHR_GCS_ARCHITECTURE.md` |
| Pi ↔ STM | **BAHR-LINK**: ikili çerçeveler, UART 115200, XOR sağlama. Şimdilik özel protokol; MAVLink'e geçiş Faz 13'te yeniden değerlendirilecek (bkz. §7) | `firmware/reflex/Core/Inc/pi_link.h` (yetkili kaynak), `MAVLINK_PROTOCOL.md` |
| Sensör ölçümleri | Her ölçüm `zaman damgası + değer + kalite + geçerlilik + kaynak` taşır | `SENSOR_INTERFACE.md` |

## 5. Faz haritası (yeniden eşlenmiş)

Prompt'un 30 fazı korunuyor ama sahibi ve sırası mimari düzeltmesine göre
değişti. **Sıra güvenlik önceliğine göre** (prompt §57: güvenlik > gerçek
zamanlı kontrol > sensör bütünlüğü > kestirim > navigasyon > görev >
batimetri): önce STM'nin güvenlik fazları, sonra sensör yolu, sonra Pi.

| Prompt fazı | Sahibi | Bu projede | Durum |
|---|---|---|---|
| 0 Analiz | — | `ARCHITECTURE_REVIEW.md` | ✓ |
| 1 Mimari + arayüzler | — | bu belge, `SENSOR_INTERFACE.md` | ✓ |
| **9** Arm / güvenlik | STM | arm durum makinesi, ön-arm kontrolleri | ✓ |
| **8** Motor kontrolü | STM | karıştırıcı, rampa, sınır, ölü bant | ✓ |
| 3 Zaman | STM + Pi | µs saat, ölçüm damgası, Pi'de saat ofseti | ✓ |
| 2 Donanım soyutlama | STM | ortak ölçüm/durum arayüzü | ayrı yapılmadı (yalnızca `SENSOR_INTERFACE.md`) |
| 4 IMU + attitude | STM → Pi | BNO086: quaternion + jiroskop + ivme, zaman damgalı akış | ✓ |
| 5 GNSS | **Pi** | fix sınıflandırma, doğruluk, geçerlilik | ✓ |
| 6 EKF / INS | **Pi** | konum/hız + yön füzyonu, ölü hesap | ✓ |
| 7 RC + manuel | STM | ✓ (SBUS, mod anahtarı, Pi'siz sürüş) | ✓ |
| 10 Waypoint nav | **Pi** | merkezi koordinat kütüphanesi, XTE/ATE | ✓ (XTE/ATE'yi Faz 11 kullanacak) |
| 11 Hat takibi | **Pi** | Pure Pursuit + algoritma arayüzü | ✓ |
| 12 Yön/hız PID | **Pi** | PID, motor komutuna bağlama | ✓ (kazançlar yer tutucu tekneye göre) |
| 13 ROS 2 köprüsü | Pi | Docker'da Jazzy; BAHR-LINK→MAVLink kararı | ertelendi |
| 14 ROS 2 navigasyon | Pi | — | ertelendi |
| 15 Görev durum makinesi | Pi | — | ertelendi (GCS değişikliği gerekir) |
| 16 Tarama sistemi | Pi | hat kimliği GCS'den gelmiyor (bkz. review §4) | ertelendi |
| 17 Sonar | Pi | aralık, aykırı değer, medyan, kalite, ofset | ✓ |
| 18 Batimetri örnekleme | Pi | mesafe tabanlı örnek | ✓ |
| 19 Batimetri QC | Pi | VALID / LOW_QUALITY / INVALID | ✓ |
| 20 Failsafe | Pi + STM | yapılandırılabilir aksiyonlar | ✓ (Pi tarafı; STM'nin kendi failsafe'i değişmedi) |
| 21 Geofence + RTL | Pi | poligon geofence, güvenli RTL | ✓ (RTL yasak bölgeyi dolanmaz, geçiyorsa reddeder) |
| 22 Parametreler | Pi + STM | kalıcılık, tip/aralık, sahip | ✓ Pi tarafı (STM'ye özgü ayarlar için BAHR-LINK parametre çerçevesi eksik) |
| 23 Kalibrasyon | STM + Pi | **donanım gerekir** | bekliyor |
| 24 Tanılama | Pi | sağlık tablosu, olay seviyeleri | ✓ (simülasyonda; eşikler tahmin) |
| 25 Kayıt | Pi | görev klasörü, CSV'ler | ✓ (simülasyonda; Pi saati/RTC açık) |
| 26–27 Sim / SITL | Pi | tekne dinamiği + sensör gürültüsü + arıza enjeksiyonu | ✓ (`SITL.md`; tekne parametreleri yer tutucu) |
| 28 HIL | STM + Pi | **donanım gerekir** | bekliyor |
| 29 GCS entegrasyonu | hepsi | — | ertelendi |
| 30 Saha testi | — | **donanım gerekir** | bekliyor |

**Donanım gerektirenler** (23, 28, 30 ve tüm "donanımda doğrulandı"
maddeleri) elde cihazlar bir araya gelene kadar yapılamaz; kod ve birim/
protokol testleri bu fazların öncesinde hazırlanır.

## 6. Her faz için çalışma biçimi

Prompt §55'teki sıra korunur: gereksinim → mevcut kod → mimari → arayüz →
plan → test; sonra uygulama → birim/entegrasyon testi → derleme → statik
kontrol → belge → **PHASE COMPLETE** raporu. Raporlar
[`PHASE_REPORTS.md`](PHASE_REPORTS.md) içinde.

Doğrulama sınıfları (her raporda ayrı belirtilir):
- **Yazılım:** host'ta gerçek kod + gerçek testler (Python `pytest`, firmware
  saf mantığı host'ta derlenip çalıştırılır).
- **Derleme:** `arm-none-eabi-gcc` ile 0 hata / 0 uyarı.
- **Donanım:** gerçek cihazda — **bugüne dek yalnızca LED yakma.**

## 7. Karar kayıtları

| # | Karar | Durum |
|---|---|---|
| D1 | GNSS Pi'de (USB) | kararlı |
| D2 | Görev ve navigasyon Pi'de; STM segment izlemez | kararlı (kullanıcı düzeltmesi) |
| D3 | ROS 2 Jazzy, Docker'da | kararlı, Faz 13'te uygulanır |
| D4 | NUCLEO-G431RB, FreeRTOS yok | kararlı — STM hafif, süper-döngü yeter |
| D5 | ESC tipi (çift yönlü mü?) | **açık** — ilk enerji öncesi zorunlu kontrol |
| D6 | Sonar donanımı | açık, Faz 17 |
| D7 | Pi↔STM: özel BAHR-LINK mi MAVLink mi | BAHR-LINK ile devam; MAVLink'e geçiş Faz 13'te, host'ta pymavlink ile bayt bayt doğrulanabilir olduğu için maliyeti düşük |

## 8. Sürümleme

| Ne | Nerede | Şu an |
|---|---|---|
| Otopilot sürümü (SemVer) | `bahr_pilot/__init__.py` `__version__` | 0.1.1 |
| Pi↔STM protokol sürümü | `bahr_pilot/versions.py` `BAHR_LINK_VERSION` | Faz 3–9 çıktılarıyla artırılır |
| Görev biçimi | `bahr_pilot/versions.py` `MISSION_FORMAT_VERSION` | 1 (düz `NAV_WAYPOINT` listesi, slot 0 = ev) |
| MAVLink | v2, `common` + ArduPilot diyalekti | — |

Araç açılışında `STATUSTEXT` ile üç sürümü de bildirir.
