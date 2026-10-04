# Sensör Arayüzü

Prompt §46: *her sensör ölçümü `valid / timestamp / quality / source`
taşır; geçersiz ölçüm doğrudan kontrolcüye verilmez.* Bu belge o kuralın
iki tarafta (STM ve Pi) nasıl uygulandığını tanımlar.

## 1. Ölçüm zarfı

| Alan | Anlam |
|---|---|
| `t_us` | Ölçümün **alındığı** an, µs, STM saatinde (bkz. Faz 3). Pi kendi ölçümleri için `time.monotonic()` kullanır |
| `valid` | Ölçüm kullanılabilir mi. `false` ise `value` hiçbir yerde kullanılmaz |
| `quality` | 0–100. 0 = bilinmiyor/kötü, 100 = en iyi. Sensöre özgü eşlenir |
| `source` | Hangi sürücü/cihaz üretti (sabit numaralandırma) |
| `value` | Sensöre özgü gövde (skaler, vektör, quaternion) |

Kural: **geçerlilik tüketicide değil üreticide belirlenir**, ve geçersizlik
sessiz sıfır değil açık bayrakla taşınır (bugüne dek `battery_mv = 0`
"geçerli sıfır volt" ile "ölçülmedi"yi ayırt edemiyordu; `battery_valid`
bunu çözdü, bu belge kuralı genelleştirir).

## 2. Sürücü durumu

```c
typedef enum {
    SENSOR_OK = 0,        /* taze ve geçerli veri */
    SENSOR_NO_DATA,       /* henüz hiç ölçüm gelmedi */
    SENSOR_STALE,         /* son ölçüm zaman aşımından eski */
    SENSOR_FAULT,         /* sensör yanıt vermiyor / bozuk çerçeve */
} SensorStatus;
```

Her STM sürücüsü aynı üç çağrıyı sunar; donanıma özgü her şey sürücünün
içinde kalır, tüketici (failsafe, telemetri) hiçbir zaman sensör modeline
bağlı değildir:

```c
SensorStatus  X_GetStatus(void);        /* SENSOR_OK ise Get() güvenilir */
uint32_t      X_GetTimestampUs(void);   /* son geçerli ölçümün alındığı an */
/* + sürücüye özgü, derleme zamanında tipli okuma: IMU_Get(ImuSample *), ... */
```

BNO086 yerine başka bir IMU konursa değişen tek dosya `imu.c`'dir.

## 3. STM sürücüleri

| Sürücü | Kaynak | Zaman aşımı | Not |
|---|---|---|---|
| `sbus` (RC) | USART1, 100 kbaud 8E2 ters | 200 ms | `SBUS_IsLinkUp()` → durum |
| `imu` (BNO086) | I2C1, SHTP | 1 s | Faz 4: quaternion + jiroskop + ivme |
| `battery` | ADC1 / PA0 | 500 ms | bölücü oranı **ölçülmedi** |
| `esc` (çıkış) | TIM2 PWM | — | çıkış sürücüsü; durumu "motor sistemi" ön-arm kontrolüne girer |
| `pi_link` | USART3 | 500 ms | `PiLink_IsFresh()` → durum |

## 4. Pi sensörleri

| Sensör | Kaynak | Üretici | Doğrulama |
|---|---|---|---|
| GNSS konum/yön | RTD100, USB, Bynav `#BESTPOSA` (5 Hz, CRC32) + `#HEADINGA` | `sensors.py` | fix tipi, std. sapma, diff yaşı → kalite sınıfı (Faz 5) |
| Derinlik | echoMAP, NMEA `SDDPT`/`SDDBT`, 1 Hz | `sensors.py` | aralık, aykırı değer, medyan (Faz 17) |
| IMU | STM üzerinden BAHR-LINK | `nucleo_link.py` | STM damgası → Pi saatine (Faz 3) |

## 5. Birimler ve çerçeveler (kısa; ayrıntı Faz 10'daki `NAVIGATION.md`)

- Konum: WGS84, derece; kontrol için **ENU** (doğu, kuzey, yukarı) metre;
  MAVLink sınırında NED/derece·1e7.
- Yön: derece, **gerçek** kuzeye göre, saat yönünde (RTD100 `HEADINGA`).
  Manyetik yön (echoMAP `HCHDM`) gerçek yöne **çevrilmeden** karıştırılmaz.
- Gövde çerçevesi: x ileri, y sancak (sağ), z aşağı.
- Açılar: roll/pitch derece (IMU), yaw yalnızca GNSS ve jiroskop füzyonundan.
- Zaman: µs (STM), saniye (Pi, monoton). Duvar saati yalnızca log adlarında.
