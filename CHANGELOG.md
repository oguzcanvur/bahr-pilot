# Changelog

Bu proje [Semantic Versioning](https://semver.org/) kullanır: `MAJOR.MINOR.PATCH`.

- **PATCH** (`0.0.x`) — hata düzeltmeleri, davranış değişikliği yok
- **MINOR** (`0.x.0`) — geriye uyumlu yeni özellik
- **MAJOR** (`x.0.0`) — geriye uyumsuz değişiklik (1.0.0'a kadar bu proje için "kararlılık" anlamına gelmez, sadece kapsamlı değişiklik anlamına gelir)

Biçim [Keep a Changelog](https://keepachangelog.com/) temel alınarak tutulur.

## [Unreleased]

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

[Unreleased]: https://github.com/oguzcanvur/bahr-pilot/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/oguzcanvur/bahr-pilot/releases/tag/v0.1.0
