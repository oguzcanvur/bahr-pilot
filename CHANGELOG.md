# Changelog

Bu proje [Semantic Versioning](https://semver.org/) kullanır: `MAJOR.MINOR.PATCH`.

- **PATCH** (`0.0.x`) — hata düzeltmeleri, davranış değişikliği yok
- **MINOR** (`0.x.0`) — geriye uyumlu yeni özellik
- **MAJOR** (`x.0.0`) — geriye uyumsuz değişiklik (1.0.0'a kadar bu proje için "kararlılık" anlamına gelmez, sadece kapsamlı değişiklik anlamına gelir)

Biçim [Keep a Changelog](https://keepachangelog.com/) temel alınarak tutulur.

## [Unreleased]

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

### Bilinen eksikler
- BNO086 (IMU) henüz okunmuyor.
- Gerçek RTD100/echoMAP/Nucleo donanımıyla uçtan uca hiç test edilmedi —
  tüm doğrulama bu oturumda MAVLink/paket protokolü seviyesinde yapıldı
  (gerçek BAHR-GCS uygulaması + loopback seri port ile).
- Nucleo'dan Pi'ye batarya voltajı/IMU geri bildirimi yok.
- Donanım bekçi sayacı (watchdog) henüz etkin değil.
