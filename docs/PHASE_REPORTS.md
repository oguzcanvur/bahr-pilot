# Faz Raporları

Prompt §55'teki "PHASE COMPLETE" raporları. Sıra `ARCHITECTURE.md` §5'teki
yeniden eşlenmiş sıradır (güvenlik önce). **Doğrulama sınıfları:**
*Yazılım* = ana makinede gerçek kod + gerçek test; *Derleme* =
`arm-none-eabi-gcc` 0 hata / 0 uyarı; *Donanım* = gerçek cihazda.
**Bugüne dek donanımda doğrulanan tek şey LED yakmadır.** Aşağıdaki hiçbir
faz donanımda denenmedi.

Durum (2026-10-04): 1084 test geçiyor (ana makinede C derleyicisiyle; derleyici
yoksa C'ye dayanan testler atlanır). Firmware (en son derlendiği hâliyle) 46.1 KB
flash / 3.2 KB RAM (128 KB / 32 KB'ın %36 / %10'u), 0 hata / 0 uyarı; saf C
modüllerinde sıkı uyarı bayraklarıyla (`-Wconversion -Wshadow -Wsign-conversion …`)
0 uyarı. Faz 5, 10, 6 firmware'e dokunmaz (yalnızca Pi, Python).

---

## PHASE 0 — Repository analysis · COMPLETE
Ayrıntı: `ARCHITECTURE_REVIEW.md`, `BAHR_GCS_ARCHITECTURE.md`.
**Bulunan ve düzeltilen hatalar:** klonlanan repo çalışmıyordu (paket yolu);
`DO_CHANGE_SPEED` m/s yerine yüzde okunuyordu (AUTO'da motor %5'e düşüyordu);
`WP_RADIUS` adı/bağlantısı; AUTO'ya mod butonuyla geçişte seq 0; reboot'a yalan
`ACCEPTED`; RC ayarı flash yazımı UART kesmesinde; IWDG debugger'da reset.
**Kullanıcı düzeltmesi:** STM hafif kalır (STM = RC/manuel + motor + IMU +
güvenlik; Pi = GNSS/navigasyon/görev). İlk incelemede prompt'taki STM-ağırlıklı
bölüşüm çelişkisi gösterilmeden benimsenmişti.

## PHASE 1 — Architecture + interfaces · COMPLETE
**Dosyalar:** `ARCHITECTURE.md` (katmanlar, güvenlik katmanları, faz haritası,
karar kayıtları, sürümleme), `SENSOR_INTERFACE.md` (ölçüm zarfı, sürücü
sözleşmesi), `MAVLINK_PROTOCOL.md`, `bahr_pilot/versions.py`
(`AUTOPILOT_VERSION`, `BAHR_LINK_VERSION`, `MISSION_FORMAT_VERSION`; açılışta
`STATUSTEXT` ile bildirilir).
**Kararlar:** D1 GNSS Pi'de; D2 görev Pi'de; D3 ROS 2 Docker'da; D4 FreeRTOS yok;
D7 BAHR-LINK ile devam, MAVLink'e geçiş Faz 13'te.
**Açık:** D5 (ESC çift yönlü mü?) — ilk enerji öncesi zorunlu; D6 sonar donanımı.

## PHASE 9 — Arming + safety · COMPLETE (yazılım + derleme)
**Bulunan gerçek hata:** `armed` her tur arm anahtarının anlık konumundan
hesaplanıyordu. Kumandada arm anahtarı açıkken tekne enerjilendirilirse ilk
geçerli SBUS çerçevesi tekneyi **anında** arm ederdi (prompt: "boot sırasında
motorları enable etme").
**Dosyalar:** `arming.{h,c}` (saf durum makinesi), `failsafe.c` (yeniden yazıldı),
`tests/c/test_arming.c`, `tests/c/failsafe_harness.c`, `tests/test_failsafe_c.py`.
**Mimari:** BOOT → DISARMED → READY → ARMED. Arm yalnızca READY iken anahtarın
KAPALI→AÇIK kenarında olur; anahtar boot sonrası en az bir kez kapalı
görülmeden arm olmaz; kenarda bir kontrol başarısızsa istek reddedilir, neden
bildirilir ve anahtarın yeniden çevrilmesi gerekir (gaz sonradan merkeze gelince
kendi kendine arm olmaz). 2 sn boot bekleme süresi. RC 5 sn'den uzun kesilirse
yeniden arm gerekir; kısa kesintide ARMED kalır (failsafe motoru zaten durdurur).
Gaz-merkezde kontrolü varsayılan açık; IMU/batarya kontrolleri **varsayılan
kapalı** (ikisi de donanımda doğrulanmadı, yanlış okuma arm'ı kilitlememeli).
Pi, "PreArm: …" nedenlerini `STATUSTEXT` ile GCS'e iletir.
**Testler:** 14 saf durum makinesi testi + gerçek `failsafe.c` ile 20 senaryo
(güç-açılışta anahtar açık, boot süresi, arm sonrası rampa, ani durdurma
öncelikleri, Pi sessizliği, MANUAL'da Pi'siz sürüş, uzun RC kaybı…). Mutasyon
kontrolü: "anahtar önce kapalı görülmeli" kuralı kaldırılınca izole test
**kırılıyor**. Bu süreçte kendi ilk taslağımdaki bir hatayı testler yakaladı
(boot sırasındaki kenar harcanmıyordu).
**Kalan:** `ARMING_CHECK` bitmaskesi STM'ye taşınmıyor (Faz 22: parametre
taşıma). Donanımda arm anahtarı davranışı doğrulanmadı.
**Sonraki faz gereksinimi:** yok.

## PHASE 8 — Motor control · COMPLETE (yazılım + derleme)
**Bulunan tutarsızlık:** GCS sözleşmesinde motor 1 = **sol** (`SERVO1 =
ThrottleLeft`) ama hem STM karıştırıcısı hem Pi'nin `steer_towards`'ı motor 1'i
sancak gibi karıştırıyordu: hedef sağdayken tekne hedeften **uzaklaşırdı**.
**Dosyalar:** `motor.{h,c}`, `failsafe.c`, `navigation.py`, `vehicle.py`,
`tests/c/test_motor.c`.
**Mimari:** motor 1 sol, motor 2 sağ; direksiyon > 0 = sağa dön (sol motor hızlı).
Karıştırıcı doyma halinde iki tarafı birlikte ölçekler (dönüş oranı korunur);
çubuk ölü bandı %5; her motor için hız değişim sınırı (varsayılan %100/s, tam
ileriden tam geriye 2 sn); tavan (MOT_THR_MAX); yön ters çevirme; **durdurma
sınırsız ve anında**. Pi'nin motor çerçeveleri de aynı hız sınırından geçer:
çılgın bir Pi tahrik zincirine vuramaz.
**Testler:** 12 saf motor testi; gerçek `failsafe.c` içinde rampa/öncelik
senaryoları; Python'da `steer_towards` yönü ve GCS joystick yönü.
**Kalan:** slew/tavan/ters ayarları derlenmiş varsayılan (Faz 22'de parametre
taşıma); `MOT_THR_MIN` (ESC ölü bölgesi telafisi) uygulanmadı — ESC tipi (D5)
bilinmeden anlamsız. Hangi motorun fiziksel olarak sol olduğu ilk su
testinde belli olur; çözümü `SERVO1_REVERSED/SERVO3_REVERSED` veya uçları
değiştirmektir, sözleşmeyi değiştirmek değil.

## PHASE 3 — Time synchronisation · COMPLETE (yazılım + derleme)
**Dosyalar:** `clock.{h,c}`, `clock_core.c` (DWT çevrim sayacı → 64 bit µs),
`bahr_pilot/timesync.py` (`StmClock`), `tests/c/test_clock_core.c`,
`tests/test_timesync.py`.
**Önemli bulgu:** sistem saati HSI osilatöründen (kristal yok) → ±%1; "STM'nin
bir saniyesi" 0.99–1.01 gerçek saniye olabilir. Ofset tahmini yetmez; **sürüklenme
de** tahmin edilmeli. `StmClock` 1 sn'lik kovalarda en düşük ofseti alır (gecikme
negatif olamaz) ve tamamlanmış kovalara doğru çizgi uydurur; 32 bit sarmayı ve
STM yeniden başlamasını ele alır.
**Testler:** ideal saat, %±1 ve %0.05 sürüklenme, geç gelen çerçeveler, sarma
sınırı, STM reboot'u, ilk çerçeveler, saçma sürüklenme sınırı. Bu süreçte
tasarım kusuru bulundu: dolmamış son kova yanlış yüksek minimum veriyordu ve
çizgiyi tam ekstrapolasyon ucunda yukarı çekiyordu → tamamlanmamış kova uydurmaya
katılmıyor.
**Kalan:** *Doğruluk beklentisi:* tahmin gerçeğin, hattın görülen en küçük
gecikmesi kadar (~3 ms) gerisinde kalır; mutlak değil, aralıklar doğru.
GNSS ve sonar için ortak zaman çerçevesi (PPS yakalama) **yapılmadı**:
RTD100'ün PPS çıkışı bir STM zamanlayıcısına bağlanmadı (GNSS Pi'de).
**Sonraki faz gereksinimi:** BNO086 INT pini bağlanırsa (bir GPIO + EXTI) IMU
zaman damgası belirsizliği 20 ms'den µs'ye iner.

## PHASE 4 — IMU + attitude · COMPLETE (yazılım + derleme; donanımda test YOK)
**Dosyalar:** `imu_reports.{h,c}` (saf SH-2 rapor ayrıştırıcı), `imu.c`
(yeniden yazıldı), `pi_link.c/h` (IMU çerçevesi, TX halka tamponu),
`bahr_pilot/attitude.py`, `nucleo_link.py` (`ImuData`, `extract_frames`),
`vehicle.py` (`ATTITUDE`), `tests/c/test_imu_reports.c`, `tests/test_attitude.py`,
`tests/test_imu_link.py`.
**Kaynak:** protokol ayrıntıları hafızadan değil CEVA'nın SH-2 referans
kaynağından (`ceva-dsp/sh2`: rapor uzunluk tablosu, `sensorhubInputHdlr`,
çözücüler) okundu. **Eski sürücü hata içeriyordu:** bir SHTP paketi birden çok
rapor taşıyabilir ve yalnızca ilkini okuyordu; ikinci rapor (jiroskop) açılınca
yanlış çalışırdı.
**Mimari:** GRV (50 Hz) + kalibre jiroskop (50 Hz) + lineer ivme (25 Hz);
paketteki tüm raporlar ayrıştırılır; örnek zamanı `INT zamanı + (−timebase +
delay) × 100 µs`. STM → Pi 28 baytlık IMU çerçevesi (50 Hz): µs damgası,
quaternion Q14, jiroskop Q9, ivme Q8, geçerlilik bitleri. Pi tarafında
quaternion→roll/pitch, `AHRS_ORIENTATION` (kart yönü) uygulanır, `ATTITUDE`'a
gerçek roll/pitch ve açısal hızlar yazılır. Yaw GNSS yönünden gelmeye devam
eder (BNO086 pusulasız, yaw'ı sürüklenir).
**Düzeltilen ek hatalar:** (1) giden hat tek tamponluydu: önceki çerçeve
gidiyorken yenisi **sessizce atılırdı** → 256 baytlık halka tamponu (bütün
çerçeve ya da hiç); (2) STM alıcısı `A5 A5 5A …` dizisinde gerçek çerçeve
başlangıcını kaçırıyordu; (3) Pi çerçeve ayırıcısı bozuk sağlamada tüm çerçeve
uzunluğunu atıyordu (arkadaki gerçek çerçeveyi yutuyordu) → yalnızca bir bayt;
(4) `serial.read(64)` 64 bayt dolana kadar bekleyip çerçeve varış zamanlarını
birbirine karıştırıyordu → `in_waiting` ile.
**Testler:** SH-2 ayrıştırıcı 9 senaryo (tek/çoklu rapor, Q ölçekleri, gecikme,
rebase, kesik/bilinmeyen paket); quaternion/Euler/montaj dönüşü 21 test; IMU
çerçevesi ve ayırıcı 10 Python testi; **4 C→Python çapraz kontrol**: gerçek
`pi_link.c`'nin ürettiği IMU çerçevesini gerçek Python çözüyor, aşırı değerler
sarmak yerine doyuyor, halka taşmasında yalnızca bütün çerçeveler çıkıyor, normal
trafik hiç kaybolmuyor.
**Kalan (önemli):**
- Test paketleri **spesifikasyondan elle kuruldu, gerçek BNO086'dan yakalanmadı.**
  I2C adresi (0x4B/0x4A), eksen işaretleri, `AHRS_ORIENTATION` işaret kuralı
  donanımda doğrulanmadı.
- INT pini bağlı değil → zaman damgası ±20 ms.
- Telemetrideki `roll/pitch` artık IMU çerçevesiyle yinelenen bilgi
  (protokol sadeleştirmesi adayı).
- STM açılışta BNO086 yanıt vermese bile ilerler; `SENSOR_FAULT` durumu ve
  "IMU yok" ön-arm nedeni ayrıntılı değil (IMU kontrolü varsayılan kapalı).
**Sonraki faz gereksinimi:** Faz 5–6 (Pi): GNSS kalite sınıflandırması ve
yön/konum kestirimi bu IMU akışını tüketecek.

## PHASE 5 — GNSS · COMPLETE (yazılım; donanımda test YOK)
**Dosyalar:** `bahr_pilot/gnss.py` (sınıflandırma, geçerlilik, ayrıştırma),
`sensors.py` (tek paylaşılan `GnssState`), `state.py`, `vehicle.py`,
`tests/test_gnss.py` (65 test).
**Ne yapar:** fix türü → `NO_FIX / 2D / 3D / DGPS / FLOAT / RTK_FIXED`; her fix
ve yön ölçümü varış zamanıyla damgalanır; zaman aşımı (RTD100 1 s, echoMAP
2.5 s, yön 5 s) ve doğruluk sınırı (yatay > 10 m, yön > 10°) ile geçersizlenir;
geçersiz veri **sayı olarak değil `None` olarak** yayınlanır. echoMAP'in
manyetik yönü araç yönü olarak **kullanılmaz** (Türkiye'de sapması birkaç derece,
çerçeve karıştırmama kuralı).
**Bulunan ve düzeltilen hatalar:** (1) iki okuyucu ayrı `GnssState` kullanıp
paylaşılan araç durumunu birbirinin üzerine yazıyordu → konum ve yön RTD100'ün
RTK fix'i ile echoMAP'in zayıf GPS'i arasında **gidip geliyordu**; ayrıca
hiçbir şey bayatlamıyordu (kablo çekilse son konum sonsuza dek geçerli
görünürdü); (2) `INS_RTKFLOAT` 3D fix sayılıyordu; (3) yön santidereceye
kesilerek yuvarlanıyordu.
**Kalan (önemli):**
- `BESTPOSA/HEADINGA` alan düzeni NovAtel düzeninden alındı, **üretici
  dokümanıyla doğrulanmadı** (daha önce gerçek cihazdan ayrıştırma çalıştı, ama
  tüm alanların anlamı kanıtlanmadı).
- Alıcının Doppler hızı (BESTVELA) ayrıştırılmıyor; hız konum farkından
  kestiriliyor (Faz 6). RTD100'de açılırsa hız kestirimi iyileşir.

## PHASE 10 — Coordinate frames & geodesy · COMPLETE (yazılım)
**Dosyalar:** `bahr_pilot/geo.py`, `navigation.py`, `vehicle.py`,
`tests/test_geo.py` (31 test).
**Ne yapar:** WGS84 / ECEF / ENU / NED / BODY dönüşümleri tek yerde; tam elipsoid
(küre değil). `LocalFrame` yerel teğet düzlem (doğu, kuzey, metre),
`segment_errors` yol boyu / **yanal sapma (araç yolun sağındaysa +)**.
**Doğrulama:** bağımsız referans olarak pyproj 3.8.0, 2000 rastgele noktada:
ECEF birebir, jeodetik gidiş-dönüş < 4 nm, mesafe hatası 300 m'de 30 nm,
5 km'de 0,13 mm. Referans değerler testlere sabit olarak gömüldü (pyproj
gerekmez).
**Bulunan ve düzeltilen hatalar:** (1) `navigation.py` küresel haversine
kullanıyordu: 41°N'de **kuzey-güney km başına 1,3 m, doğu-batı km başına 2,6 m**
hata (ekvatorda 5,6 m/km) — RTK'nın 2 cm'sini boşa çıkarırdı; (2)
`LocalFrame.to_geodetic`, `to_enu`'nun tam tersi değildi (teğet düzlem elipsoidin
üstündedir): 2 km'de 0,19 mm, 5 km'de 3 mm sapma → artık nanometre; (3) pulse
hesabı `int()` ile **kesiyordu**: sayısal gürültü sağ/sol motorda 1 µs fark
yaratıyordu ve her pulse yarım µs aşağı kayıyordu → `round()`; (4) `home`
yalnızca ilk döngüde atanıyordu (ilk fix 2D ise hiç atanmazdı) → Faz 6'da
düzeltildi.
**Kalan:** `segment_errors` yazıldı ve test edildi ama henüz navigasyon
kullanmıyor (Faz 11 hat takibi kullanacak).

## PHASE 6 — State estimation · COMPLETE (yazılım; donanımda test YOK)
**Dosyalar:** `bahr_pilot/estimator.py`, `nucleo_link.py` (IMU örnek kuyruğu),
`state.py` (`pose`), `navigation.py`, `vehicle.py`, `tests/test_estimator.py`
(45 test), `tests/poses.py`, `tests/test_vehicle.py`, `tests/test_imu_link.py`.
**Mimari:** iki küçük Kalman filtresi, skaler biçimde (numpy gerekmez).
*Yön:* 2 durum (yön + jiroskop bias'ı); jiroskop ~50 Hz ile ilerletir, RTD100
çift anten yönü (1 Hz) düzeltir. Jiroskop hızı roll/pitch ile dünya çerçevesine
çevrilir `(q·sinφ + r·cosφ)/cosθ`. *Konum/hız:* her eksende sabit hız modeli,
yerel ENU'da; iki eksen tek 2×2 kovaryans paylaşır. Çıktı `Pose`: konum,
hız, rota, yön, her biri için geçerlilik ve belirsizlik.
**Güvenlik davranışları:** her ölçüm normalleştirilmiş innovation ile
kapılanır (aykırı değer denetleyiciye ulaşmaz); art arda 5 ret → filtre
**ölçüme göre yeniden başlar** (kayan filtre gerçeği sonsuza dek reddetmesin);
ilk geçerli ölçüme kadar ve zaman aşımından sonra sayı değil "bilinmiyor";
`OK → DEAD_RECKONING → NONE` (RTD100 1 s, echoMAP 2.5 s — `gnss.FIX_TIMEOUT_S`
yeniden kullanılır — sonra 5 s ölü hesap sınırı); yön ölü hesabı 10 s ile ya da
belirsizlik > 10° olunca biter (jiroskop yokken ~0,15 s, Faz 26–27'de düzeltildi); NaN/inf ve geriye giden
zaman damgası yok sayılır; 20 km'den uzak (teğet düzlemin doğruluk sınırı) fix
reddedilir. **Navigasyon artık ham alıcı değerlerini hiç kullanmaz**, yalnızca
`state.pose`'u (bir test bunu güvence altına alır).
**Gecikme:** alıcıların çıkış gecikmesi **ölçülmedi.** İlk taslak bunu
ölçüm gürültüsüne ekliyordu; duman testinde konum σ'sı 1,5 m/s'de 0,19 m
çıktı (RTK'nın cm hassasiyeti boşa gidiyordu). Şimdi nominal gecikme
(0,1 s) telafi ediliyor (ölçüm `hız × gecikme` kadar ileri taşınıyor) ve yalnızca
gecikmenin **belirsizliği** (0,05 s) gürültüye ekleniyor.
**Araca bağlama:** `GLOBAL_POSITION_INT` artık süzülmüş konum + hız (vx kuzey,
vy doğu, cm/s) ve yön; `GPS_RAW_INT` alıcının ham değeri (rota alanı gerçek
rota, önceden yön yazılıyordu); `VFR_HUD` hızı kestirimden. Eski iki noktalı
hız hesabı (fix tekrar ederken hız 0'a düşüyordu) kaldırıldı.
**Testler:** sentetik tekne simülasyonu (gürültülü + bias'lı jiroskop, gecikmeli
GNSS). Yön: işaret kuralı, kuzey ve güney sarma (güney, innovation sarmasının
asıl kritik yeri), jiroskopla taşıma (tutma RMS > 3° iken filtre < 1°), bias
kestirimi, ölü hesap süreleri, aykırı değer, kilitlenmeme. Konum: yakınsama,
rota pusula açısı, gecikme telafisi (telafisiz yanlılık > 12 cm, telafiyle <
4 cm), ölü hesap programı, kapı, yeniden başlama, kaynak başına zaman aşımı,
echoMAP 1 Hz'in "ölü hesap" sayılmaması. **Bağımsız tutarlılık:** NEES (gerçek
hataya karşı) ≈ 1,2 → filtrenin raporladığı belirsizlik dürüst; NIS ≈ 0,6
(gecikme payı yüzünden temkinli). **Mutasyon kontrolü:** 10 kasıtlı bozma
denendi (jiroskop işareti, bias çıkarma, iki kapı, gecikme telafisi,
kovaryans güncelleme, sarma, süreç gürültüsü, ölü hesap sınırı, yeniden
başlama); 1'i ilk başta yakalanmadı (innovation sarması — sürekli simülasyon 180°
geçişini yalnızca bir kez görüyordu), **deterministik sınır testi eklenerek
kapatıldı.** Ek bulgu: ilk yazdığım dört test beklentisi yanlıştı (nominal rota
yerine gerçek rota, aritmetik yerine dairesel ortalama…); filtre değil test
düzeltildi, her biri sayılarla ayrıştırıldı.
**Ölçülen doğruluk (sentetik, donanım değil):** ivme 0,1 m/s²'de konum RMS
0,086 m, hız RMS 0,19 m/s. Hatanın baskın terimi **varsayılan gecikme
belirsizliği** (0,05 s × 1,5 m/s = 7,5 cm).
**Sonradan (Faz 26–27 simülatörü) bulunan ve düzeltilen yön hataları:** jiroskop
arızasında tahmin edilen yön 167–180° yanlış olduğu hâlde geçerli kalıyordu
(tekne kendi dönüşünü görmeyip dönüyordu). Düzeltmeler: bilinmeyen dönüş hızı
varsayımı 10 → 60°/s ve belirsizliğin zamanın karesiyle büyümesi, GNSS yönüyle
çelişen yönün geçersiz sayılması, jiroskop zaman aşımı 0,3 → 0,1 s. Ayrıntı:
Faz 26–27 raporu. Bu yüzden yukarıdaki "yön ölü hesabı ... jiroskop yokken ~1 s"
ifadesi artık ~0,15 s'dir.
**Kalan (önemli):**
- **Tüm gürültü ve gecikme değerleri tahmin** (jiroskop gürültüsü, bias yürüyüşü,
  ivme gürültüsü 0,5 m/s², iki gecikme). Gerçek BNO086/RTD100 verisiyle
  ayarlanmalı: jiroskop için Allan varyansı, alıcı gecikmesi için sabit hızla düz
  seyir veya PPS karşılaştırması. **Gecikme belirsizliği ölçülünce konum
  hatası belirgin düşer.**
- Alıcının kendi Doppler hızı kullanılmıyor (Faz 5 notu).
- Gecikme telafisi tek sabit; durum geçmişi tamponu (gecikmeli ölçüm
  güncellemesi) yapılmadı — gecikme ölçüldükten sonra gerekirse.
- İvmeölçer kullanılmıyor (yaw'ı belirsiz; sabit hız modeli yeterli görüldü).
- Gerçek IMU zaman damgası belirsizliği ±20 ms (INT pini bağlı değil).

## PHASE 26–27 — Simulation / SITL · COMPLETE (yazılım; tekne parametreleri yer tutucu)
**Dosyalar:** `bahr_pilot/sitl/{boat,sensors,harness}.py`, `tests/test_sitl.py`
(37 test), `tests/test_sitl_vehicle.py` (22 test), `docs/SITL.md`.
**Ne yapar:** tekne dinamiği (3 serbestlik derecesi, suya göre hız, akıntı,
rüzgâr, firmware'in rampası, ESC/pervane gecikmesi), sensör modelleri ve 9 çeşit
arıza enjeksiyonu; gerçek `Vehicle`, `Estimator` ve `Navigator` kapalı döngüde
koşar. 5 dakikalık görev ~2,5 s. Ayrıntı ve **kullanım sınırları: `SITL.md`**.
**Önemli sınır:** tekne parametrelerinin **hepsi yer tutucu** (`%60 gaz ≈ 1,5 m/s`
çıkacak şekilde seçildi). Simülatör kontrol *mantığını* sınar (işaretler,
sınırlar, arıza davranışı), kazanç ayarlamaz.
**Doğrulama:** fizik, **kapalı formda çözülebilen kararlı durumlarla**
karşılaştırıldı (uç hız ve saf dönüş hızı ikinci derece denklemlerin kökleri,
rüzgârda yana kayma), simülatörün kendi çıktısıyla değil. Ayrıca: integrasyon
**ikinci derece** (her adım yarılamada fark dörtte bire iniyor); rampa tam
geriden tam ileriye 2 s; durdurma anlık; geri itki daha zayıf; kablolama hatası
ters döndürüyor; sensörler yapılandırılmış istatistiği tutturuyor (σ, gecikme,
hız, kuzeyden sarma). Üç mutasyon denemesi (eski hatalar geri getirilerek) **yalnızca
simülasyon senaryolarıyla da** yakalandı.
**Simülatörün bulduğu hatalar (hepsi düzeltildi):**
1. **Jiroskop arızasında yön, doğru olmadığı hâlde "geçerli" kalıyordu.** Jiroskop
   donunca ya da geçersiz olunca tahmin edilen yön 167–180° yanlıştı ve geçerli
   sayılıyordu; tekne tam diferansiyel itkiyle ~38°/s dönerken yönünü göremeyip
   **kontrolsüz dönüyordu** (yanlış-ama-geçerli örnek: 425 / 991 tick). İki sebep:
   (a) jiroskopsuzken belirsizlik, bilinmeyen dönüş hızı için 10°/s varsayıyordu
   (tekne 38°/s dönebiliyor) → `max_yaw_rate_dps = 60`; (b) GNSS yönü art arda
   reddedilirken (25 ret) filtre kendi donmuş yönünü geçerli sayıyordu → **çelişen
   yön artık geçerli sayılmıyor**. Sonuç: yanlış-ama-geçerli 9 / 16 tick, en kötü
   hata 21° / 17°.
2. **Bilinmeyen dönüş hızı belirsizliği döngü hızına bağlıydı.** Her adımda
   `(σ·dt)²` ekleniyordu: hızı her adımda bağımsız gürültü sayar, doğrusal büyür
   ve tick hızına bağlıdır (yön ~3× fazla uzun geçerli kalıyordu). Sabit ama
   bilinmeyen hız için doğrusu `(σ·T)²` → düzeltildi, tick hızından bağımsızlığı
   bir test doğruluyor.
3. Jiroskop zaman aşımı 0,3 s'den 0,1 s'ye indirildi (örnekler 20 ms'de bir gelir;
   0,3 s boyunca son hızla ekstrapolasyon güvensiz).
4. Simülatörün kendi hatası: motor gecikmesi rampanın adım *sonu* değeriyle
   sürülüyordu → birinci derece integrasyon (20 s'de 3,5 mm); ortalamayla
   sürülerek ikinci dereceye çıkarıldı.
**Taban ölçümü (mevcut waypoint-to-waypoint yönlendirme, gerçek `Vehicle`,
yer tutucu tekne; yalnızca görev boyunca, XTE = çizgiye dik sapma):**

| senaryo | ort. \|XTE\| | p95 | en fazla |
|---|---|---|---|
| sakin su, 100 m'lik bacaklar | 0,37 m | 1,07 m | 2,99 m |
| sakin su, uzun bacaklar | 0,40 m | 0,93 m | 2,99 m |
| yan akıntı 0,2 m/s | 2,98 m | 5,51 m | 5,59 m |
| **yan akıntı 0,5 m/s** | **7,23 m** | 13,29 m | **13,51 m** |
| rüzgâr 15 N | 4,13 m | 7,58 m | 7,69 m |
| GNSS 20 s kesik | 0,72 m | 2,19 m | 3,10 m |
| GNSS 3D'ye düşmüş (σ 2,5 m) 30 s | 0,43 m | 1,04 m | 4,86 m |

Yorum: sakin suda köşe kesme `WP_RADIUS` (3 m) ile sınırlı; **yan akıntı/rüzgârda
sapma çok büyük çünkü hedef yönüne dönmek çizgiyi hiç hedeflemiyor**. Faz 11
(çizgi takibi) ve Faz 12 (PID) bunu bu tabloyla karşılaştırarak iyileştirmeli.
**Kayıtlı sınırlamalar** (`SITL.md`): kablolama hatası fark edilmiyor (Faz 23),
donmuş jiroskop GNSS yönüne ≤1 s sonra anlaşılıyor, anten hizasızlığı fark
edilmiyor (Faz 20 rota-yön tutarlılığı), HOLD = nötr komut (konum tutmuyor).
**Kalan:** sensör modelinde IMU zaman damgası belirsizliği (±20 ms) yok;
ivmeölçer, batarya, STM failsafe'i, RC modellenmedi.

## PHASE 11 — Path following · COMPLETE (yazılım; yer tutucu tekneyle ölçüldü)
**Dosyalar:** `bahr_pilot/pathfollow.py` (yeni), `navigation.py`, `vehicle.py`
(`NAVL1_*`, `NAV_CONTROLLER_OUTPUT`), `tests/test_pathfollow.py` (21 test),
`tests/test_navigation.py`, `tests/test_vehicle.py`, `tests/test_sitl_vehicle.py`.
**Sorun:** eski yönlendirme **hedefe doğru** dönüyordu; yan akıntı/rüzgârın
ittiği tekne hedefe bakarken çizgiden kayıyordu (simülatörde 0,5 m/s akıntıda ort. 7,2 m,
en fazla 13,5 m sapma). Tarama teknesinin çizgide kalması gerekir.
**Çözüm:** *Pure pursuit.* Tekne çizgiye izdüşürülür (yol boyu / yanal sapma),
çizgi üzerinde `lookahead` kadar ilerideki noktaya yönelir. Öne bakış mesafesi
ArduPilot'un L1 formülü: `L = sönüm × periyot × yer hızı / π` (3–30 m ile
sınırlı), yani GCS'de zaten bulunan gerçek `NAVL1_PERIOD` (10 s) ve
`NAVL1_DAMPING` (0,75) parametreleri ArduPilot kullanıcısının beklediği gibi ayar
yapar. Algoritma bir `PathFollower` arayüzü arkasında (Stanley/L1 değiştirilebilir;
testte eski "hedefe dön" davranışı aynı arayüzle takılıyor). Çizgi, başlangıcına
sabitlenmiş yerel teğet düzlemde yaşar. Bacak, ikinci görev maddesinden itibaren
**önceki waypoint'ten** başlar (köşede ne olursa olsun planlanan çizgi izlenir);
ilk madde, GUIDED ve RTL'de teknenin o andaki konumundan. Köşe: waypoint ya
yarıçap içine girince ya da hedef düzlemi `2 × WP_RADIUS` içinde geçilince
"varıldı" (güçlü akıntı tekneyi yarıçapın hemen dışında tutup waypoint etrafında
döndürmesin). `NAV_CONTROLLER_OUTPUT` artık gerçek hedef yönünü ve yanal sapmayı
(`xtrack_error`, sağ +) yolluyor. `crosstrack_gain` (prompt'un `NAV_CROSSTRACK_GAIN`'i)
yapılandırmada; parametre olarak henüz açılmadı (Faz 22).
**Ölçülen etki (aynı senaryo, aynı ölçüt, yer tutucu tekne; |XTE| ort. / en fazla):**

| senaryo | önce | sonra |
|---|---|---|
| sakin su | 0,37 / 2,99 m | 0,36 / 2,97 m |
| yan akıntı 0,2 m/s | 2,98 / 5,59 m | **0,53 / 2,96 m** |
| yan akıntı 0,5 m/s | 7,23 / 13,51 m | **1,07 / 4,00 m** |
| rüzgâr 15 N | 4,13 / 7,69 m | **0,70 / 3,63 m** |
| GNSS 20 s kesik | 0,72 / 3,10 m | 0,48 / 3,06 m |
| GNSS 3D'ye düşmüş 30 s | 0,43 / 4,86 m | 0,45 / 3,01 m |

Sakin sudaki ~3 m en fazla `WP_RADIUS` (3 m) ile sınırlı köşe kesmesi.
**Testler:** geometri **elle türetilmiş** değerlerle (çizginin yanı, kuzey/doğu
bacak, çapraz hat, çizgi öncesi/sonrası, sıfır uzunluk, varış kuralları, L1
formülü, kinematik yakınsama); navigator bacak yönetimi (ilk bacak teknenin
olduğu yerden, sonrakiler önceki waypoint'ten, hedef değişince yeniden kurulma,
5 km'lik bacakta mm doğruluk); simülasyonda **eski yöntemle yan yana** karşılaştırma
(aynı tekne ve bozucu; yalnızca takipçi farklı: pure pursuit ort. sapmayı ≥ 3,3×
küçültüyor), kısa `NAVL1_PERIOD` çizgiyi daha sıkı tutuyor, güçlü akıntıda waypoint
etrafında dönme yok. **Mutasyon kontrolü:** 8 kasıtlı bozma (öne bakış işareti,
sapma ağırlığı, varış kuralı, öne bakış ucu aşıyor, alt sınır, bacak başlangıcı,
hedefe dönme, bacak yenilenmiyor) — hepsi yakalandı.
**Kalan:**
- Kazançlar (öne bakış) yer tutucu tekneyle; gerçek tekne verisiyle ayarlanmalı.
- Sabit akıntıda sabit bir yanal sapma kalır (tekne ~20° yengeç açısıyla gitmek
  zorunda, bunu çizgiden ~1,3 m uzakta yapıyor): çözüm adayı, ölçülen rota ile yön
  farkından yengeç açısı telafisi veya yanal sapma integrali (`NAVL1_XTRACK_I`).
- `WP_PIVOT_ANGLE` (keskin dönüşte yerinde dönme) yok.

## PHASE 12 — Heading and speed control · COMPLETE (yazılım; yer tutucu tekneyle ayarlandı)
**Dosyalar:** `bahr_pilot/control.py` (yeni), `navigation.py`, `vehicle.py`
(`ATC_*` parametreleri), `estimator.py` (yaw hızı bias düzeltmeli), `sitl/boat.py`
(`motor_gain`), `gcs/param_meta.py` (`ATC_STR_ANG_P`), `tests/test_control.py`
(29 test), `tests/test_sitl_control.py` (33 test), `tests/test_navigation.py`.
**Yapı ve parametre adları ArduRover'ınkiler** (GCS'de zaten etiketli):
*Direksiyon:* yön hatası `--ATC_STR_ANG_P-->` istenen dönüş hızı (`ATC_STR_RAT_MAX`
ile sınırlı) `--PID + ileri besleme (ATC_STR_RAT_P/I/D/FF/FILT), ölçülen yaw hızına
karşı-->` direksiyon, −1..+1. *Hız:* gaz = ileri besleme (`CRUISE_THROTTLE` @
`CRUISE_SPEED`) + yer hızı hatası üzerinde PID (`ATC_SPEED_P/I/D`); istenen hız
`ATC_ACCEL_MAX` ile rampalanır. Viraj yavaşlatması (büyük yön hatasında hız
kesilir) korundu. Karıştırıcı firmware `Motor_Mix` ile aynı kural (doygunlukta
iki taraf birlikte küçülür, dönüş **oranı** korunur; firmware testindeki
vakalarla aynı sayılar sınanıyor).
**PID ayrıntıları:** türev ölçüm üzerinde (hedef sıçrayınca tekme yok); integral
sınırlı ve çıkış doygunken donuyor (anti-windup); geçersiz zaman adımı (0, geri, > 0,5 s,
NaN) hiçbir zaman büyük integral/türev üretmez; NaN girdi her şeyi sıfırlar;
`filter_hz ≤ 0` filtresiz demektir (sıfıra bölme yok). **Tekne sürülmediği anda
(konum/yön geçersiz, hedef yok, varıldı) tüm integraller ve rampa sıfırlanır.**
Parametreler GCS'den (ya da bozuk bir mesajdan) geldiği için `configure()` her değeri
güvenli aralığa zorluyor; NaN → varsayılan, 0 Hz filtre → 0,5 Hz.
**Ayar (yer tutucu tekneyle, tek tekneye aşırı uyumdan kaçınarak):** ilk tahminim
(ANG_P 1,5; P 0,30; I 0,08; FF 0,70) yön adımında **%24–53 aşım ve 10–16 s oturma**
verdi. Kazançlar, **üç teknenin en kötüsü** (nominal; ağır-zayıf: kütle ×1,5, atalet
×1,6, itki ×0,7; hafif-güçlü) üzerinde 20°/60°/120° adımlarıyla aranarak seçildi;
ızgaranın kenarında çıkan ilk sonuç genişletilerek doğrulandı. Seçilen:
`ATC_STR_ANG_P 0,75; RAT_P 0,9; RAT_I 0,03; RAT_D 0; RAT_FF 0,3; ATC_SPEED_P 0,25; I 0,10`.
Öğrenilenler: yüksek açı kazancı aşım yaratıyor; direksiyon integrali **küçük** olmalı
(`I ≥ 0,04` büyük dönüşlerde birikip oturmayı 18 s'ye çıkarıyor) **ama gerekli**: bir
motor %30 zayıfken `I = 0` ortalama sapmayı 0,42 m, `I = 0,03` 0,29 m, `I = 0,08` 0,20 m
yapıyor (eşit motorlarda bedeli yok; `I = 0,03` aşım/oturma ile dayanıklılık arasında
seçildi). Hız PI altı çiftten en iyisi.
**Ölçülen sonuçlar (yer tutucu tekne):**
- Yön adımı, 9 durumun (3 tekne × 20°/60°/120°) en kötüsü: **%10,3 aşım**, 10,5 s'de ±2°
  (ağır-zayıf tekne, 120° dönüş).
- Hız: 0→1,5 ve 1,5→0,8 m/s adımları 3 teknede ≤ 8,7 s'de oturuyor (en kötü aşım
  %18, hafif-güçlü teknede: ileri besleme gazı onun için fazla). Ters akıntıda yer hızı
  1,502 m/s'de tutuluyor.
- **Dayanıklılık taraması** (tekne yanlışsa): kütle 0,5–2×, atalet 0,3–3×, itki 0,6–1,4×,
  sürtünme 0,5–2× aralığında 10 farklı teknenin **tamamı görevi tamamlıyor**; sakin
  suda ort. sapma ≤ 0,34 m, 0,4 m/s akıntıda ≤ 1,05 m.
- Görev tablosu (|XTE| ort. / en fazla; sütunlar: eski hedefe-dönme → Faz 11 → Faz 12):

| senaryo | başlangıç | Faz 11 | **Faz 12** |
|---|---|---|---|
| sakin su | 0,37 / 2,99 | 0,36 / 2,97 | **0,10 / 2,99** |
| yan akıntı 0,2 m/s | 2,98 / 5,59 | 0,53 / 2,96 | **0,38 / 2,96** |
| yan akıntı 0,5 m/s | 7,23 / 13,51 | 1,07 / 4,00 | **0,88 / 2,97** |
| rüzgâr 15 N | 4,13 / 7,69 | 0,70 / 3,63 | **0,52 / 3,00** |
| GNSS 20 s kesik | 0,72 / 3,10 | 0,48 / 3,06 | **0,10 / 3,01** |
| GNSS 3D'ye düşmüş 30 s | 0,43 / 4,86 | 0,45 / 3,01 | **0,21 / 2,95** |

(~3 m en fazla `WP_RADIUS` ile sınırlı köşe kesmesi.)
**Mutasyon kontrolü:** 12 kasıtlı bozma (anti-windup, integral sınırı, türev hata
üzerinde, rampasız hız, doygunsuz karıştırıcı, yön işareti, yaw hızı yok sayılıyor,
sarma yok, negatif gaz, sıfırlama yok, viraj yavaşlatması yok, parametre korumasız);
ilk turda **3'ü kurtuldu**: biri benim hatalı mutasyonumdu, ikisi gerçek test
zayıflığıydı (negatif gaz testi PID'i hiç negatife götürmüyordu; viraj yavaşlatması
testi dümen doygunluğu iki motoru zaten küçülttüğü için **yanlış nedenle geçiyordu**).
Testler güçlendirilince 12'si de yakalandı.
**Kalan (önemli):**
- **Tüm kazançlar yer tutucu tekneye göre**; gerçek teknede adım yanıtı, dönüş çemberi
  ve durma mesafesi ölçülüp yeniden ayarlanmalı. Dayanıklılık taraması bunun
  başlangıç noktası olarak güvenli olduğunu gösteriyor, ayarlanmış olduğunu değil.
- Sabit akıntıda ~1 m sabit yanal sapma (yengeç açısı) hâlâ var (Faz 11 notu).
- `ATC_STR_RAT_FILT`, `i_max`, viraj yavaşlatma eşikleri sabit; Faz 22'de parametre olur.
- Hız kontrolcüsü ileri beslemeye bağlı; yanlış `CRUISE_THROTTLE` (%18 aşım) PI ile
  telafi ediliyor ama yavaş.
- SITL testleri artık ~100 s sürüyor (tam takım ~3 dk).

## PHASE 20 — Failsafe · COMPLETE (yazılım; donanımda test YOK)
**Dosyalar:** `bahr_pilot/failsafe.py` (yeni, saf mantık), `vehicle.py`,
`sitl/harness.py`, `tests/test_failsafe_monitor.py` (47 test), `tests/test_vehicle.py`,
`tests/test_sitl_vehicle.py`. (STM'nin kendi failsafe'i — `failsafe.c`, kumanda kaybı
ve Pi bayatlığı — ayrı ve değişmedi.)
**Kaynaklar ve aksiyonları seçen ArduRover parametreleri (hepsi GCS sözlüğünde
zaten vardı):** GCS bağlantı kaybı (`FS_GCS_ENABLE`, `FS_TIMEOUT`, `FS_ACTION`),
düşük batarya (`BATT_FS_ENABLE`, `BATT_LOW_VOLT`, `BATT_FS_LOW_ACT`), kritik batarya
(`BATT_CRT_VOLT`, `BATT_FS_CRT_ACT`), konum kaybı ve yön kaybı (`FS_EKF_ACTION`).
Aksiyonlar: rapor (0), RTL (1), HOLD (2), sonlandır = disarm (5); SmartRTL (3, 4) için
kayıtlı iz gerekir, yok: 3 → RTL, 4 → HOLD; bilinmeyen/bozuk değer HOLD.
**Kurallar:** her koşul tetiklenmeden önce bekleme süresi (batarya 3 s — yük çökmesi
failsafe değil; konum 1 s) ve temizlenmeden önce 2 s yokluk ister (titreyen girdi
operatörü boğmasın, modu çırpmasın); batarya için **histerezis** (motorlar durunca
paket toparlanıp kendi failsafe'ini silmesin); **seviye tetiklemeli**: failsafe
sürdükçe özerk modda kalınamaz (operatör nedeni sürerken yeniden AUTO seçerse
yine HOLD); neden geçse bile görev **kendiliğinden devam etmez**, operatör yeniden
devreye alır; en güçlü aksiyon kazanır (sonlandır > HOLD > RTL > rapor); konum ve
yön yalnızca özerk modlarda izlenir (MANUAL'de pilot seyirci), GCS ve batarya
silahlıyken her modda izlenir ama özerk olmayan mod hiç değiştirilmez; **hiç
duyulmamış bir GCS "kaybolmuş" sayılmaz** (ArduPilot gibi); yön, jiroskop yokken
GNSS yönü geldiği hızda (~0,15 s/s) titrediği için **kesintisiz geçersizlikle değil
bir pencerede erişilebilirlikle** (≥ %30, ≥ 5 s veriyle) yargılanıyor. RTL için ev
konumu şart (ArduRover gibi reddediyor), failsafe RTL istediğinde ev yoksa HOLD'a
düşüyor ve söylüyor.
**Davranış değişikliği (bilinçli):** HOLD artık gerçekten **modu değiştiriyor**.
Eskiden GCS kaybında motor nötrleniyor ama mod AUTO'da kalıyordu: bağlantı geri
gelince görev **sürpriz biçimde kendiliğinden devam ediyordu**. Ayrıca düşük batarya
ve konum kaybı olay olarak bildiriliyor (`STATUSTEXT`), yön kaybı (IMU çıkarsa) ilk kez
görünür bir mesaj üretiyor.
**Simülatörün bulduğu iki sorun (ikisi düzeltildi):**
1. **Test altyapısı yanlış nedenle geçiyordu.** `Flight.completed` "mod HOLD oldu"
   demekti; failsafe de HOLD yaptığı için GNSS kesintisi sonrası "görev devam etti"
   testleri, tekne bacak 0'da (0,1; 100,6) duruyorken geçiyordu. Tamamlanma artık "son
   waypoint'e gerçekten varıldı"; operatör eylemleri (yeniden AUTO) ve yayınlanan mesajlar
   da izleniyor.
2. **GCS bağlantı yaşı duvar saatiyle hesaplanıyordu** (`time.time()`), diğer her şey
   benzetim zamanıyla: simülasyonda failsafe hiç tetiklenmiyordu. Araç artık `gcs_last_seen`'i
   monotonik saatte tutuyor ve yaşı failsafe'e verilen `now` ile hesaplıyor (bir mutasyon
   testi bu saat uyumsuzluğunu yakalıyor).
Üçüncü ayar: başlangıçta GNSS yönünün ilk gelişini beklemek kayıp sayılmasın diye
yön penceresi 6 s, asgari veri 5 s, eşik %30 (jiroskopsuz ~%15, jiroskoplu ~%100).
**Testler:** kurallar elle (zaman değerleriyle) sınanıyor: bekleme süreleri, histerezis,
titreyen bağlantı (bir "triggered", araya "cleared" girmiyor), yeniden tetikleme, güçlü
aksiyonun kazanması, mod kapsamı, hiç duyulmayan GCS, yön erişilebilirliği (jiroskopsuz
~%15 → kayıp; %60 → sorun yok; başlangıç gecikmesi → sorun yok), disarm sıfırlar.
Araç seviyesinde: HOLD'a geçiş ve mesajlar, görev kendiliğinden devam etmiyor, nedeni
sürerken AUTO tekrar seçilince yine HOLD, RTL/ev yok → HOLD, sonlandır disarm eder,
rapor-only uçmaya devam eder, MANUAL değişmez, düşük batarya. Simülatörde: GCS sessiz →
HOLD ve operatör bekleniyor; `FS_ACTION=1` tekneyi eve getirip HOLD'da bırakıyor; rapor-only
görevi bitiriyor; ölü jiroskop → HOLD, **tekne dönmüyor, duruyor** (Faz 6/26'daki
kontrolsüz dönme artık gerçekten kapalı); kısa GNSS kesintisi failsafe üretmiyor.
**Mutasyon kontrolü:** 14 kasıtlı bozma (histerezis, bekleme süresi, temizleme
bekleme süresi, en zayıf aksiyonun kazanması, bilinmeyen değerin rapor sayılması, her
modda izleme, yön hiç kayıp sayılmıyor, 1 s veriyle yargılama, motorlar hiç durmuyor,
seviye tetiklemesiz, sonlandır disarm etmiyor, RTL geri düşüşü yok, ev olmadan RTL,
farklı saat) — 14'ü de yakalandı.
**Kalan:**
- `FS_THR_*` (kumanda kaybı) STM'de; `FS_CRASH_CHECK` yok (çarpışma/sıkışma algılama yazılmadı).
- Sonlandır yalnızca Pi tarafını disarm eder; STM'nin arm durumu kumandanın arm anahtarına bağlı.
- Rota-yön tutarlılığı (anten hizasızlığı, Faz 6 notu) ve kablolama hatası (Faz 23) hâlâ
  izlenmiyor.
- Batarya failsafe'i varsayılan KAPALI (gerilim bölücü ölçülmedi); `BATT_FS_ENABLE=0` iken
  hiçbir batarya failsafe'i çalışmaz.
- Bekleme süreleri sabit (`failsafe.py` başında adlandırılmış); Faz 22'de parametre olur.

## PHASE 21 — Geofence and safe RTL · COMPLETE (yazılım; donanımda test YOK)
**Dosyalar:** `bahr_pilot/geofence.py` (yeni, saf geometri), `failsafe.py` (`GEOFENCE`
kaynağı), `vehicle.py` (MAVLink çit protokolü, tahmine dayalı kontrol, RTL yol
denetimi), `tests/test_geofence.py` (38 test), `tests/test_failsafe_monitor.py`,
`tests/test_vehicle.py`, `tests/test_sitl_vehicle.py`; GCS tarafında
`gcs/param_meta.py` (`FENCE_*` girdileri) ve `gcs/setup_window.py` (Geofence sayfası).
**Önce bir hata:** araç, görev protokolünde `mission_type` alanını **hiç okumuyordu**.
QGC ya da Mission Planner bir geofence yükleseydi (tip 1) araç onu görev listesi sanıp
**gerçek görevin üzerine yazardı**. Artık tip 0 (görev, davranış aynen), tip 1 (çit),
tip 2 (rally: açıkça reddedilir, yanlış dosyalanmaz) ayrı ele alınıyor. Tip 0'da
`MISSION_CLEAR_ALL`'a yanıt **bilerek** verilmiyor: BAHR-GCS yüklemeden önce temizleyip
sonra gelen her `MISSION_ACK`'ı "araç erken onayladı" sayıp bir adımı atlıyor; yanıt
eklemek GCS'nin görev yüklemesini bozardı (bir test bunu sabitliyor).
**Üç tür sınır, her biri tek bir sayıya iner (sınıra işaretli mesafe, içerisi +):**
ev etrafında daire (`FENCE_RADIUS`, `FENCE_TYPE` biti 2), yüklenen **dahil** alanları
(araç herhangi birinin içinde olmalı), yüklenen **hariç** alanları (hepsinin dışında
olmalı: yasak bölge, kablo geçişi). Çokgen (5001/5002) ve daire (5003/5004) standart
MAVLink çit öğeleriyle yüklenir. Geometri yerel teğet düzlemde (`geo.py`, elipsoid
doğruluğu). İhlal = en küçük payın negatif olması; bir sayı olması histerezis ve kesin
geometri testi sağlıyor. Geçersiz çit (3'ten az köşe, tutarsız köşe sayısı, sıra
bozuk, yarıçap ≤ 0, NaN, kapsam dışı koordinat) reddedilir ve **eski çit korunur**.
**Failsafe kaynağı:** `GEOFENCE` (`FENCE_ENABLE`, `FENCE_ACTION`, `FENCE_MARGIN`);
0,5 s bekleme; `FENCE_MARGIN` aynı zamanda temizleme histerezisi (sınır boyunca giderken
açılıp kapanmasın). Varsayılan **kapalı**: çizilmemiş bir çit tekneyi durdurmasın.
**Tahmine dayalı kontrol (simülatör gösterdi):** tekne durduğu yerde duramaz. Çit hem
konumda hem de tekne *şimdi durursa duracağı noktada* (hız yönünde) denetleniyor, küçük
pay geçerli: çitten uzaklaşan tekne tetiklemez, yaklaşan durma mesafesi kadar önce
tetikler. Durma mesafesi = reaksiyon (1 s: döngü + kestirim + STM'nin %100/s rampası +
ESC/pervane gecikmesi) + süzülme (`FENCE_COAST_DECEL`, ortalama süzülme yavaşlaması). İlk
varsayımım yanlıştı: `v²/(2·ATC_ACCEL_MAX)` komutlanan frenlemeyi varsayıyor, ama tekne
**aktif frenlemez, süzülür** ve sürtünme düştükçe yavaşlama da düşer: 1,5 m/s'den süzülme
4,2 m (kapalı form: ln(1 + c₂v/c₁)/c₂), 1 m değil. Ayrı bir parametre yapıldı
(`FENCE_COAST_DECEL`, yer tutucu 0,3 m/s²); **gerçek tekne için "motoru kes, kaç metrede
durdu" testiyle ölçülmeli.**
**Ölçülen aşım (1,5 m/s, yer tutucu tekne), çitin dışına:** tahminsiz 2–6 m → tahminle
≤ 1,0 m (HOLD) ve RTL'de **hiç aşmıyor** (en kötü pay +0,7 m). RTL tekneyi çevirip eve
getiriyor ve HOLD'da bırakıyor; HOLD sınırda duruyor; yalnızca rapor eden çitin dışına
geçiyor (141 m, çit 120 m).
**Güvenli RTL:** RTL düz çizgi eve gider ve bu çizgi yasak bölgeden geçebilir. RTL'den
önce yol 1 m aralıkla taranır: yasak bölgeden **çıkan** yol serbest, tekrar **giren** ya da
izinli suya hiç ulaşmayan ya da eve (izinli suda olmayan) varan yol değil → RTL reddedilir
(`RTL refused: path to home crosses the fence`); failsafe zaten HOLD'a düşüyor. İnce bir
yasak şerit (3 m) atlanmıyor. Ev konumu yoksa RTL reddedilir (ArduRover gibi).
**Operatör davranışı:** çit ihlali sürerken AUTO yeniden seçilirse tekrar HOLD olur; HOLD'da
çitin dışında kalan tekne **kumandayla** geri sürülmeli (GUIDED de HOLD'a zorlanır). Çit
yüklenirken ya da görev yüklenirken çitin dışında kalan waypoint'ler için `STATUSTEXT`
uyarısı verilir; çit yüklü ama `FENCE_ENABLE` 0 ise bunu da söyler.
**Testler:** düzlem geometrisi elle (kare/L/daire pay değerleri, yön bağımsızlığı, içbükey
çokgen girintisi); birden çok dahil alan (birlik) ve hariç alan (kesişim); en küçük payın
kazanması ve neyin kazandığı; her tür yükleme hatası ve eski çitin korunması;
MAVLink protokolü (istek sırası, ACK, görevin dokunulmaması, alan yok = tip 0, geri
indirme, boş yükleme, rally ret, tiplenmiş temizleme); tahmin (dışa giden/içe giden/duran/
yavaş tekne); kestirim kullanılıyor (ham fix değil); simülatörde HOLD/RTL/rapor, yasak alan,
dahil alan, tahminin etkisi (kapatılınca aşım > 2×), operatör çit dışında devam ettiremiyor.
**Mutasyon kontrolü:** 18 kasıtlı bozma (dahil alanın birlik yerine kesişim olması, hariç
payın işareti, dış çokgenin pozitif sayılması, daire işareti, köşe sayısı sınırı, slot 0,
`mission_type` yok sayılıyor (eski hata), tahmin yok, tahminin konumu ezmesi, tip 0'a
yanıt, ham fix, rally kabul, histerezis yok, bekleme yok, RTL yeniden-giriş, RTL son nokta,
kaba örnekleme, RTL yol denetimi yok) — 18'i de yakalandı.
**Kalan:**
- Çit yalnızca RAM'de (Faz 22).
- RTL düz çizgi; yasak bölgeyi **dolanmaz**, geçiyorsa reddeder ve HOLD'a düşer (gerçek
  SmartRTL için izlenen yolu kaydetmek gerekir; yapılmadı).
- Sınırın kendisi (`FENCE_COAST_DECEL`, reaksiyon süresi) yer tutucu tekneye göre.
- GCS'de çit çizme/yükleme arayüzü yok (yalnızca parametre sayfası); çit şimdilik QGC/Mission
  Planner ile yüklenir (Faz 29).
- Yükseklik çiti anlamsız (tekne); `FENCE_TYPE` bit 1/8 yok sayılır.

## PHASE 22 — Parameters (ranges, persistence) · COMPLETE (Pi tarafı; STM tarafı eksik)
**Dosyalar:** `bahr_pilot/params.py` (yeni), `vehicle.py`, `tests/test_params.py` (143 test),
`tests/test_vehicle_params.py` (29 test).
**Bulunan üç sorun (doğrulandı, düzeltildi):**
1. **Hiç doğrulama yoktu.** `PARAM_SET` her sayıyı, `NaN` dahil, kabul ediyordu.
   `FS_TIMEOUT = NaN` olursa `yaş > NaN` hiçbir zaman doğru olmaz: **GCS failsafe'i sessizce
   kapanırdı**. `RCMAP_THROTTLE = 0` STM'ye −1 numaralı kanal gönderirdi.
2. **Hiçbir şey kalıcı değildi.** Her yeniden başlatma PID kazançlarını, çiti, failsafe
   ayarlarını varsayılana döndürüyordu.
3. **Açılışta Pi, RAM'deki *varsayılan* RC haritasını STM'ye yollayıp** STM'nin flash'ta
   sakladığı kalibrasyonun üzerine yazıyordu.
**Tasarım:** tüm 74 parametre için aralık / tür / küme tablosu (`SPECS`); bir değer ancak
sonlu bir sayıysa, aralıktaysa, tamsayı olması gerekiyorsa tamsayıysa, numaralandırmaysa
izinli kümedeyse kabul edilir. Reddedilen `PARAM_SET` eski değeri korur ve **eski değer geri
yankılanır** (GCS aracın gerçekten tuttuğunu görür) + `STATUSTEXT` neden. Kabul edilenler
JSON dosyasına **atomik** (geçici dosya + `fsync` + `os.replace`) ve son değişiklikten 1 s
sonra yazılır (GCS onlarca parametreyi arka arkaya yazar); yalnızca **varsayılandan farklı**
olanlar saklanır, böylece yazılım güncellemesindeki yeni varsayılan hiç değiştirilmemiş
parametreler için geçerli olur. Yüklemede her giriş aynı doğrulamadan geçer: bozuk dosya
`.corrupt` olarak kenara alınır ve varsayılanlar kullanılır; tek tek bozuk girişler atılır ve
raporlanır, geri kalanı korunur; format uyuşmazlığı uyarıyla okunur; yazılamayan disk bir kez
bildirilir ve yeniden denenir. `MAV_CMD_PREFLIGHT_STORAGE`: 0 dosyadan yeniden yükle, 1 şimdi
kaydet, 2 varsayılana sıfırla (dosyayı siler, RC haritasını STM'ye yeniden yollar). `SIGTERM`
(systemd) kapanışta son bir kayıt yapar. `--param-file` (varsayılan `~/.bahr_pilot/params.json`).
**STM'ye açılış itmesi:** artık yalnızca bu Pi'de gerçekten **kaydedilmiş** RC değerleri varsa
yapılır; yoksa STM kendi flash'ındaki haritayı korur (Pi onu geri okuyamaz).
**Önemli:** her varsayılan kendi aralığında geçerli olmak zorunda (74 test), bir parametre
aralıksız eklenemez (`ParamStore` oluşturulurken hata), aralıklar `Navigator.configure`'ın
sanitize aralıklarıyla aynı (savunma iki katmanlı).
**Testler:** değer kontrolü tablosu (NaN/inf/None/str/bool, aralık, tamsayı, küme), set()
(kabul/ret/yuvarlama/bilinmeyen/paylaşılan tablo), yazma (sessizlik süresi, yalnızca
farklılar, varsayılana dönünce dosyadan çıkma, yol yok, dizin oluşturma, artık dosya yok,
başarısız yazmada eski dosyanın bozulmaması ve yeniden deneme), yükleme (yeniden başlatmada
kalıcılık, yeni varsayılanın ulaşması, bozuk dosya 5 çeşit, tek tek atılan girişler, format),
sıfırlama; araçta: ret yankısı, **NaN `FS_TIMEOUT` failsafe'i artık kapatamıyor (uçtan uca)**,
reddedilen RC parametresi STM'ye gitmiyor, yeniden başlatmada kalıcılık, kaydedilmiş
değerlerin denetleyicilerce kullanılması, STM'ye açılış itmesi (kayıt yok → itme yok), 
`PREFLIGHT_STORAGE` (yaz/yazamayan disk/dosya yok/sıfırla/yeniden yükle/bilinmeyen),
`main()` varsayılan yolu. **Ana döngü sırası** da ilk kez test edildi (tahmin → ev → failsafe →
kayıt → kumanda modu → motor): döngüden bir adım düşse önceki hiçbir test kırılmazdı.
**Mutasyon kontrolü:** 16 kasıtlı bozma (NaN kabulü, aralık, tamsayı, küme, reddedilen değerin
saklanması, her değerin kaydedilmesi, atomik olmayan yazma, sessizlik süresi, bozuk dosyanın
kenara alınmaması, bozuk girişlerin kabulü, STM'ye koşulsuz itme, reddedilen RC'nin itilmesi,
yankısızlık, sıfırlamada itmeme, döngüde kayıt/failsafe yok) — 16'sı da yakalandı.
**Kalan (önemli):**
- **STM tarafı parametreleri yok:** `MOT_SLEWRATE`, `MOT_THR_MAX`, `ARMING_CHECK` gibi
  firmware'in kullandığı ayarlar şu an derlemede sabit; GCS'den değiştirmek için BAHR-LINK'e
  genel bir parametre çerçevesi + flash ayar kaydının genişletilmesi gerekir (firmware işi).
- Pi, STM'nin flash'taki RC haritasını **geri okuyamaz**. Pi dosyası kaybolur ve STM'de
  kalibrasyon varsa, GCS tek bir RC parametresi yazdığında Pi tüm haritayı (diğerleri varsayılan)
  yollar ve STM'dekini ezer. Çözüm BAHR-LINK'te geri okuma ister.
- Tüm parametreler `REAL32` (tür bilgisi yok); GCS aralığı kendi sözlüğünden alır.
- Aralıklar elle yazıldı; GCS sözlüğüyle çapraz doğrulaması yok (bahr_pilot ayrı repo).
- Dosya yalnızca Pi'de; araç değişince ya da SD kart yenilenince kaybolur (yedek yok).

## PHASE 17 — Sonar · COMPLETE (yazılım; gerçek sonar verisiyle test YOK)
**Dosyalar:** `bahr_pilot/sonar.py` (yeni, saf mantık), `tests/test_sonar.py` (37 test). Araca
bağlanması Faz 18'de.
**Ne yapar:** ham derinlik okumalarını (echoMAP ~1 Hz) süzer ve kalite verir: GOOD / SUSPECT / BAD.
Tek ışınlı ekolotların tipik kötü okumaları: dip kilidi yok (0 ya da en büyük menzil), tepe (balık,
yosun, kabarcık, elektriksel parazit), gürültülü kısa süreler. Her biri haritaya sahte bir tepe/çukur
olarak geçerdi.
1. **Menzil:** `RNGFND1_MIN..RNGFND1_MAX` (ArduPilot adları) dışı: BAD.
2. **Tepe:** son kabul edilen okumalardan geçen doğrunun (Theil-Sen, tek kötü noktaya dayanıklı)
   *şu an* için öngördüğü değerden `max(SONAR_SPIKE 0,5 m, derinliğin %5'i)` fazla sapan: BAD.
3. **Basamak:** üst üste 3 reddedilen okuma birbiriyle uyuşuyorsa dip gerçekten değişmiştir: yeni
   seviye benimsenir (ilk onaylayan SUSPECT, sonrakiler GOOD; önceki iki okuma BAD kalır, sonradan
   düzeltilmez).
4. **Gürültü:** kabul edilse de, son 10 yeniliğin (okuma − öngörü) MAD-tabanlı σ'sı 0,4 m'yi
   aşarsa SUSPECT. Başlangıç ya da boşluk sonrası ilk 3 okuma geçmişsiz olduğu için SUSPECT.
`depth_m` = ham okuma + `RNGFND1_OFFSET` (dönüştürücünün su hattı altı derinliği), **yumuşatılmaz**
(yumuşatma, anketin amacı olan araziyi bulanıklaştırır); BAD için `None`. Ekstrapolasyon son kabul
edilen okumadan en fazla 3 s ileri (eğim gürültüsü uzun sessizlikle büyümesin), 5 s'den uzun
sessizlik geçmişi unutturur.
**Tasarımı değiştiren bulgu (ölçerek):** ilk sürüm okumayı son 5'in *medyanıyla* karşılaştırıyordu. Dik
bir şevde (25° eğimde 1,5 m/s → saniyede 0,63 m) her okuma medyandan toleranstan fazla sapar ve ardışık
reddedilenler birbiriyle uyuşmadığı için basamak da onaylanmaz: **şevin tamamı reddediliyordu**.
Medyan yerine doğrusal eğilim öngörüsüne geçince şev kaybolmuyor. Bununla birlikte göreli tolerans
%20'den %5'e indirildi: %20, 8–14 m'de 1,6–2,8 m'lik tepeleri geçiriyordu (sahte tepelerin %13'ü).
**Ölçülen performans (benzetilmiş ekolot: 0,1 m çözünürlük, σ 0,05 m, %8 tepe ±1–5 m, %3 dip-kilidi-yok
sıfırı; 10 tohum):**

| dip | geçen sahte tepe | reddedilen iyi okuma | geçirilen verinin RMS hatası |
|---|---|---|---|
| düz 8 m | %0,4 | %0,1 | 0,074 m |
| hafif eğim 0,01 m/s | %0,4 | %0,1 | 0,075 m |
| dik şev 0,5 m/s | %1,8 | %0,1 | 0,079 m |

**Testler:** menzil sınırları ve geçersiz girdi (NaN/inf/None/str/bool), ısınma, iki yöne tepe,
tepenin sonrasını etkilememesi, göreli/mutlak tolerans, basamak (3 uyuşan okuma; aşağı ve yukarı;
uyuşmayanlar hiç onay vermez; 2 uyuşan yetmez), **dik eğim reddedilmez**, eğimdeki tepe yine
yakalanır, sırt (eğim dönüşü: 2 okuma reddedilir, 3.'sü yeni eğilimi benimser), gürültü, ofset
(menzil ham okumaya uygulanır), kısa/uzun boşluk, ekstrapolasyon sınırı, sıra dışı zaman damgası,
simülasyon istatistikleri. **Mutasyon:** 12 kasıtlı bozma (medyana dönüş, sınırsız ekstrapolasyon,
basamak yok, uyuşmayanlar birlikte sayılıyor, ofset yok, iki menzil sınırı, bool kabulü, boşluk
unutturmuyor, gürültü işaretlenmiyor, ısınma yok, yalnızca mutlak tolerans) — 12'si de yakalandı.
**Kalan (önemli):**
- **Gerçek echoMAP verisiyle denenmedi**; eşikler (0,5 m, %5, 0,4 m, 3 okuma) tahmin. Gerçek
  ekolotun gürültüsü ve balık/yosun davranışıyla ayarlanmalı.
- **Sırtın ya da sığlık tepesinin zirvesinde 2 okuma (~3 m) kaybolur** (bir sırt "basamak" gibi
  öğrenilir). Sığlık tespiti için önemli olabilir; ham okumalar bayraklarıyla kaydedileceği için
  (Faz 25) hiçbir şey kalıcı kaybolmaz, ama haritada o noktalar boş görünür.
- 1 okuma genişliğinde gerçek bir cisim (kaya) tepeden ayırt edilemez.
- Yalnızca derinlik süzülür; eğim ve açıklık-yankı (footprint) düzeltmesi, ses hızı düzeltmesi,
  gel-git düzeltmesi yok (son ikisi tipik olarak sonradan işlemdir).
- Sonar donanımı (D6) hâlâ açık: echoMAP'in tek ışınlı olması varsayıldı.

## PHASE 18 — Bathymetry sampling · COMPLETE (simülasyonda gerçek tabanla doğrulandı; gerçek ekolotla YOK)
**Dosyalar:** `bahr_pilot/bathymetry.py` (yeni), `sensors.py` (derinlik kuyruğu), `estimator.py`
(`Pose.roll_deg/pitch_deg`), `vehicle.py`, `sitl/sensors.py` + `sitl/harness.py` (ekolot modeli),
`tests/test_bathymetry.py` (43), `tests/test_vehicle_bathymetry.py` (24),
`tests/test_sitl_bathymetry.py` (15).
**Ne yapar:** sonar süzgecinin okumalarını konumla birleştirip **kat edilen mesafeye göre**
örnekler (saniyeye göre değil): her `BATHY_SPACING` (1 m) metrede bir örnek; duran tekne (< 0,2 m/s)
hiç örnek üretmez (yığılmasın), hızlı tekne boşluk bırakmaz. Reddedilen okumalar da (INVALID, nedenle)
kaydedilir ve aralık sayacını sıfırlamaz: günlükte "neden delik var" görünür. **Yankının
konumu:** tahmin edilen konum, okumanın yaşı (işlem anı − okuma anı) + ekolotun kendi gecikmesi
(`SONAR_LATENCY`, ölçülmedi) kadar hız yönünün tersine geri alınır (5 s ile sınırlı). Her örnek hem
su hattı altı derinliği (ham + `RNGFND1_OFFSET`) hem ham okumayı, konum doğruluğunu, GNSS kalitesini,
hızı, yönü, yatmayı taşır: sonradan işleme (gelgit, ses hızı, başka eşik) sınıflamayı yeniden yapabilir.
**Araca bağlama:** `GnssState` artık **her** derinlik okumasını kuyruğa alıyor (eskiden yalnızca en
sonuncusu tutuluyordu); echoMAP aynı sondajı SDDPT ve SDDBT olarak iki kez yolluyor, 0,3 s içinde gelen
ikinci okuma çift sayılır ve atılır. Pi yalnızca silahlıyken ankete başlar (tezgâhtaki tekne anket
yapmaz). **GCS çıkışı düzeltildi:** `DISTANCE_SENSOR` eskiden her telemetri turunda (5 Hz) *ham* değeri
tekrarlıyordu; BAHR-GCS her mesajda haritaya nokta eklediğinden aynı sondaj 5 kopya çiziliyor ve tepeler
doğrudan geçiyordu. Artık kabul edilen her sondaj için **bir** mesaj, süzülmüş + ofsetli değerle,
ısınma okumaları hariç (bir sahte tepe bu yolla geçmişti).
**Parametreler:** `RNGFND1_MIN/MAX/OFFSET` (ArduPilot adları), `SONAR_SPIKE`, `SONAR_LATENCY`,
`BATHY_SPACING`; hepsi aralık denetimli, kalıcı (Faz 22), GCS sözlüğünde Türkçe açıklamalı.
**Simülatörde (100 m'lik anket hattı, 26° şev, durgunluktan hızlanarak):** temiz ekolotla 0 INVALID,
5 LOW_QUALITY (ısınma ve şevin dirseği), 63 VALID; kaydedilen derinlik, kaydedilen konumdaki gerçek
tabana RMS **0,060 m**, en kötü 0,15 m; örnekler sırayla, saniyede bir, ~1,5 m aralıkla. **Gecikme
telafisi etkisi:** `SONAR_LATENCY` 0 iken RMS 0,111 m (en kötü 0,26 m), 0,2 iken 0,060 m: yankı raporlanmadan
0,2 s önce alınır, 0,5 m/m'lik şevde 1,5 m/s'de bu 0,15 m derinlik hatasıdır.
**Simülatör bir filtre kilitlenmesi buldu (Faz 17'ye geri düzeltme):** durgunluktan hızlanıp şeve girerken
eğim *artıyor*; 5 noktalık eğilim bunu takip edemedi, sapma toleransı aştı, pencere donunca ardışık
reddedilenler (her biri öncekinden 0,7 m farklı) birbirine "uymadı" ve **12 okuma (~20 m) üst üste**
INVALID kaldı. Düzeltmeler: gerçek bir eğim varken ikinci (hızlı) öngörücü; ardışık reddedilenlerin
sabit seviye değil **doğru** üzerinde olabilmesi; 6 art arda retten sonra filtrenin yeniden eşitlenmesi.
Sonuç: temiz ekolotta 0 INVALID. Ayrıntı: Faz 17 raporu.
**Testler:** mesafeye göre örnekleme (aralık, ilk örnek, yürüyen tekne 0,3 m/s'de metre başına bir, durağanda
hiçbir şey, reddedilenler sayacı sıfırlamıyor, çok yakın düşürülüyor), konum kaydırma (yaş + gecikme,
5 s sınırı, hız yönü), kayıt alanları, kuyruk (iki cümle tek sondaj, yalnız SDDBT, bozuk alanlar, 64
sınırı), araç (silahlıyken örnekler, silahsızken yalnız tile, ısınma gönderilmiyor, 5 yerine 1 mesaj,
ofset ve menzil, 16 bit sınırı, ayar değişince filtre yeniden başlıyor, aynı ayar başlatmıyor,
min>max güvenli, iyi sonra kötü okuma), simülasyon anketleri. **Mutasyon:** 20 kasıtlı bozma (aralık yok,
reddedilen aralığı sıfırlıyor, durağan kayıt, konum kaydırma yok, yaş sınırı yok, DR güvenilir, doğruluk /
hız / yatma denetimi yok, yatma yalnız roll, invalid → low, tek neden, yön geçersizken, bench'te anket,
her turda gönderme, ısınma / reddedilen GCS'ye, çift cümle, döngüden düşme) — 20'si de yakalandı.
**Kalan (önemli):**
- Gerçek echoMAP ve gerçek konum akışıyla denenmedi; `SONAR_LATENCY` ve gecikmenin gerçek değeri ölçülmeli.
- Örnekler henüz diske yazılmıyor (yalnızca son 2000 RAM'de): Faz 25.
- Ses hızı, gelgit, ışın genişliği (footprint) ve belirsizlik (TPU) hesabı yok.
- Sadece tek ışın ve dikey derinlik; eğik yankı geometrisi yalnız bayrak olarak.

## PHASE 19 — Bathymetry QC · COMPLETE (aynı modül ve testler)
**Sınıflama (her örnek için tek bir sonuç, tüm nedenler kayıtlı):**

| sonuç | ne zaman |
|---|---|
| **INVALID** | sonar okuması BAD (menzil dışı, tepe, sayı değil) ya da konum yok |
| **LOW_QUALITY** | sonar SUSPECT (ısınma, basamak onayı, gürültü); konum ölü hesapla; konum doğruluğu `BATHY_MAX_HACC`'tan (0,5 m) kötü ya da bilinmiyor; hız `BATHY_MAX_SPEED`'ten (3 m/s) fazla; yatma `BATHY_MAX_TILT`'ten (15°) fazla |
| **VALID** | hiçbiri |

INVALID her zaman LOW nedenlerini de taşır. **Konum doğruluğu**, tahminci filtrenin kendi 1-σ'sıdır
(kalite sınıfına göre tabanı var: RTK 0,03; float 0,25; DGPS 1,0; standalone 2,5 m), yani standalone
GNSS ile bir koşu otomatik olarak LOW_QUALITY çıkar (simülasyonda doğrulandı). **Yatma**, dikeyden
sapmadır `acos(cos roll · cos pitch)` (roll 12° ve pitch 10°'nin ayrı ayrı eşik altında olup birleşik
15,5° olması yakalanıyor).
**Simülatörde:** tepe ve dip kilidi yok okumalarıyla (%8 + %3) 13 INVALID (≈ enjekte edilen 12), kullanılabilir
örneklerde **en kötü hata 0,12 m** (hiçbir sahte tepe VALID ya da LOW'a sızmıyor); tüm koşu standalone
GNSS ile LOW_QUALITY; 15 s sonar kesintisi günlükte boşluk bırakıyor, failsafe üretmiyor, kesinti
sonrası filtre ısınma okumalarını LOW işaretleyerek yeniden başlıyor; sonarı hiç yanıt vermeyen tekne yine
görevi tamamlıyor. **Eşikler (0,5 m, 3 m/s, 15°) tahmin**; gerçek ekolot, tekne ve ölçüm gereksinimine
(ör. IHO S-44 sınıfı) göre ayarlanmalı.
**Kalan:** IHO S-44 gibi bir standart için belirsizlik bütçesi (derinlik + konum + sensör) hesaplanmıyor;
sınıflama eşik tabanlı. Yatma yalnızca bayrak, düzeltme değil.

## PHASE 25 — Mission logging · COMPLETE (simülasyonda doğrulandı; gerçek SD kartla YOK)
**Dosyalar:** `bahr_pilot/missionlog.py` (yeni), `vehicle.py` (bağlama), `sitl/harness.py`,
`tests/test_missionlog.py` (26), `tests/test_vehicle_missionlog.py` (12), `tests/test_sitl_bathymetry.py` (+1).
**Ne yapar:** `--log-dir` verilmişse her **görev** (silahlan → silahı bırak) için
`<log-dir>/missions/mission-YYYYMMDDTHHMMSS[-n]/` klasörü açar; içinde düz CSV/JSON dosyaları (Excel, QGIS,
betik açar):

| dosya | içerik |
|---|---|
| `meta.json` | yazılım / BAHR-LINK / görev biçimi sürümleri, **tüm parametreler** ve varsayılandan farklı olanlar, başlatma argümanları (log klasörü hariç), mod, home, görev nokta sayısı |
| `bathymetry.csv` | Faz 18'in ürettiği **her** örnek: konum, derinlik, ham derinlik, VALID / LOW_QUALITY / INVALID, tüm nedenler, sonar kalitesi, GNSS kalitesi, σ, hız, yön, yatma, yol boyunca mesafe. Reddedilenler de var: veri setindeki delik açıklanabilir |
| `track.csv` | 2 Hz: tahmin edilen konum, hız, rota, yön (+geçerlilik), konum durumu, σ, roll/pitch, dönüş hızı, mod. Konum bilinmiyorsa satır yazılmaz |
| `events.csv` | operatöre söylenen her şey (failsafe, mod değişimi, silahlanma, uyarı) zaman damgası ve önem derecesiyle; `STATUSTEXT` ve `set_mode` otomatik düşer |
| `summary.json` | görev bitince: süre, **mesafe**, nokta sayıları, kalite başına örnek sayısı, kullanılabilir örneklerin derinlik aralığı, önem derecesine göre olay sayısı |

**Dayanıklılık (SD kart dolar, takılır, çekilir):** her satır yazılır yazılmaz işletim sistemine gider
(elektrik kesilirse en çok yazılmakta olan satır kaybolur), 5 saniyede bir ve sonunda `fsync`; **herhangi bir
yazma hatası** kaydı durdurur, `error`'a ve `dropped` sayacına yazılır, **asla fırlatılmaz**: araç döngüsü diskin
yüzünden ölmez. Operatör GCS'de tek bir kez "Log: write failed, logging stopped" görür. `summarize_folder`,
elektrik kesilmesi yüzünden `summary.json`'u olmayan bir klasörü verilerinden yeniden özetler. Aynı saniyede iki
görev klasörü çakışmaz (`-2`). Kapanışta (SIGTERM dahil) açık görev `shutdown` nedeniyle kapatılır.
**Simülasyonda gerçek veriyle bulunan iki hata (ikisi de düzeltildi):**
1. *Her görevin ilk örneği günlükte yoktu.* Döngüde `_update_bathymetry`, klasörü açan `_update_mission_log`'dan
   **önce** çalışıyordu; silahlanma turunda üretilen örnek henüz açık olmayan günlüğe gidip sessizce düşüyordu.
   Sıra değiştirildi (günlük önce) ve döngü sırası testi buna göre sabitlendi.
2. *Özetteki mesafe şişiyordu.* Simülasyonda gerçek yol **102,0 m** iken özet **104,7 m** dedi: 44 saniye durup
   2 Hz'de toplanan konum titreşimi 2,7 m "yol" oldu. Şimdi mesafe yalnızca son *sayılan* noktadan en az 0,25 m
   uzaklaşılınca ilerliyor (`DISTANCE_MIN_STEP_M`); çapa her adımda değil yalnızca sayılınca taşındığı için
   0,3 m/s ile sürünen tekne durmuş sayılmıyor (iki ayrı testle sabitlendi). Ölçüm: günlükteki mesafe,
   simülatörün kendi gerçek yoluyla ±1 m içinde.
**Simülatörde (100 m anket hattı, 110 s, `--log-dir` ile gerçek `Vehicle`):** `bathymetry.csv` satırları
`bathy_samples` ile birebir aynı sırada ve eksiksiz; özet kalite sayıları örneklerle aynı; özet derinlik aralığı
kayıtlı örneklerden yeniden hesaplananla aynı; `track.csv` ~2 Hz (220 nokta); olaylarda `mode -> HOLD` ve
`Logging to mission-…`; `meta.json` değişen parametreyi (`SONAR_LATENCY` 0,2) taşıyor.
**Testler:** satır biçimleri, boş / bilinmeyen alanlar, iki Hz sınırı, konum yokken yazmama, olay önem
derecesi adları, özet (mesafe, derinlik aralığı **konum yok → INVALID** örnekler dışarıda, olay sayıları),
yazma hatası (kayıt durur, fırlatmaz, bir kez raporlanır), aynı saniyede iki görev, yeniden başlatma, çökmüş
klasörü yeniden özetleme; araçta: silahlanma klasör açar ve söyler, silahı bırakma özetle kapatır, silahsızken
hiçbir şey, her silahlanma ayrı görev, track 2 Hz, STATUSTEXT/mod olay olur, failsafe olay günlüğüne düşer,
örnekler yazılır, meta, bozuk disk bir kez raporlanır, `--log-dir` yoksa günlük yok, kapanışta kapatma.
**Mutasyon:** 22 kasıtlı bozma (21'i üretim kodunda, 1'i simülatör adaptöründe), 22'si de yakalandı:
STATUSTEXT / mod olay değil, örnek yazılmıyor, silahı bırakınca kapanmıyor, track yok, hata her turda / hiç
raporlanmıyor, kapanışta açık kalıyor, ikinci silahlanma aynı görev, operatöre klasör söylenmiyor, döngüde sıra
ters / adım yok, simülatörde adım yok, meta her parametreyi "değişmiş" sayıyor / log klasörünü sızdırıyor,
titreşim mesafe sayılıyor, çapa her adım taşınıyor, reddedilen derinlik aralığa giriyor, konumsuz track, track
sınırsız, yazma hatası kaydı durdurmuyor, aynı saniye çakışması. İlk turda 4 bozma sağ kalmıştı: üçü benim zayıf
testimdi (yanlış metni sayan hata testi; derinlik aralığı testinde derinliği olan INVALID örnek yoktu); dördüncüsü
yalnızca simülatör adaptöründeki sırayı değiştiriyordu: üretim kodu değil ve gerçek döngü sırasını zaten döngü
sırası testi sabitliyor (bozma onu yakaladı), o yüzden listeden çıkarıldı.
**Kalan (önemli):**
- **Pi'nin saati.** Pi 4'te gerçek zamanlı saat (RTC) yok; `utc` sütunları ve klasör adları `time.time()`'a, yani
  sistem saatine bağlı. Sahada internet/NTP yoksa bu saat büyük olasılıkla yanlış (bu Pi'de hangi saat kaynağının
  kullanıldığı **denetlenmedi**). Gelgit düzeltmesi için `utc` güvenilir olmalı: RTD100'ün GNSS zamanından
  (NMEA GGA'daki saat tarihsiz; Unicore günlük başlığındaki GPS haftası/ms tarihi de verir) sistem saati ayarlanmalı
  ya da örneklere GNSS UTC'si yazılmalı. **Yapılmadı.** Monotonik `t` aynı koşunun günlüklerini birbirine bağlamaya
  yetiyor.
- Eski görev klasörleri hiç silinmiyor, boş alan denetimi yok (disk dolunca yalnızca kayıt durur ve bir kez
  bildirilir).
- Gerçek SD kartta yazma/`fsync` yükü ve gecikmesi ölçülmedi (Pi 4 döngüsü 20 Hz; satır başına flush ucuz olmalı,
  ama ölçülmedi).
- Günlük okuma / QGIS dışa aktarma aracı yok (dosyalar standart CSV/JSON).

## PHASE 24 — Diagnostics (health table) · COMPLETE (simülasyonda; donanımda YOK)
**Dosyalar:** `bahr_pilot/diagnostics.py` (yeni), `vehicle.py` (bağlama), `nucleo_link.py`, `failsafe.py`
(mesaj düzeltmesi), `sitl/harness.py`, `tests/test_diagnostics.py` (88), `tests/test_vehicle_health.py` (31),
`tests/test_sitl_health.py` (4), `tests/test_nucleo_link.py` (+1), `tests/test_failsafe_monitor.py` (+2).
**Ne yapar:** "teknede ne bozuk?" sorusuna tek cevap. 11 parça (GNSS, yön, IMU, STM bağlantısı, RC, batarya,
ekolot, GCS bağlantısı, depolama, ana döngü, failsafe) her biri **OK / UYARI / HATA**:

| seviye | anlamı | örnekler |
|---|---|---|
| UYARI | işini yapıyor ama bozuk | ölü hesap, sessiz ekolot, düşük batarya, kumanda sinyali yok, yavaş döngü, STM yeni yeniden başladı |
| HATA | gereken bir işlev yok | konum yok, geçerli yön yok, IMU verisi yok, STM sessiz, kritik batarya, görev günlüğü yazamıyor |

Bu bir **rapor, eylem değil**: motoru durdurmaz, mod değiştirmez (onu `failsafe.py` yapar; nedenleri tabloda
`failsafe` satırı olarak görünür ve ikinci kez duyurulmaz).
**Üç çıktı:** (1) her seviye değişimi `STATUSTEXT` (UYARI 4, HATA 3, düzelme 6), metin `Parça: neden`
(`GNSS: no position`, `battery: 10.0 V low`, `STM link: restarted (1)`) ve buradan görev günlüğünün olay dosyasına;
(2) `SYS_STATUS.onboard_control_sensors_present/enabled/health` bitleri (eskiden 0, 0, 0) ve `load` = ana
döngünün doluluğu (‰); (3) arm anındaki tablo `meta.json`'da. **BAHR-GCS bit alanlarını okumaz**
(`_system_status_name` hep "OK"); QGroundControl / Mission Planner gösterir. `HEARTBEAT.system_status` hâlâ hep
`ACTIVE`: BAHR-GCS onu "failsafe" satırında gösterdiği için dokunulmadı.
**Titreşim ve gürültü önlemi (tablo ≠ duyuru):** tablo yalnızca debounce'u izler: kötüleşme 1 s sürmeli, düzelme
3 s. *Duyuru* ayrıca beklemeli: açılıştan sonraki ilk 10 s hiçbir şey söylenmez (GNSS ve ekolot uyanıyor; bu sürede
düzelen arıza hiç anılmaz) ve bir parça en çok 5 s'de bir konuşur, her duyuru operatörü **tablonun o anki haline**
getirir, araya giren çırpınmaları tek tek tekrarlamaz. Tablonun kendisi duyuruyu beklemez, yani `SYS_STATUS`
açılışta "sağlıklı" yalan söylemez (ilk taslakta duyuru bekleyişi tabloyu da geciktiriyordu; `SYS_STATUS`
bağlanırken fark edilip ikisi ayrıldı).
**Eşikler nereden geliyor (keyfî değil):** STM bayatlığı = `RC_TELEMETRY_FRESH_S` (1 s); ekolot sessizliği =
`SonarConfig.gap_s` (5 s, süzgecin geçmişi unuttuğu an); döngü hatası = tahmincinin `imu_timeout_s`'i (0,1 s);
döngü uyarısı = bir 20 Hz tik (0,05 s); IMU kare hızı 50 Hz nominalin %80'inin altı (BAHR-LINK belgesi); batarya
histerezisi failsafe'inkiyle aynı (0,4 V) ki tablo ile failsafe "düzeldi" konusunda hemfikir olsun; GCS uyarısı 2 s
(FS_TIMEOUT varsayılanı 3 s'den önce). Bunların birbirine bağlı kaldığını bir test denetler.
**Simülatörde ölçülen (gerçek `Vehicle` döngüsü, `--log-dir` açık):**

| senaryo | ne zaman | ne söylendi |
|---|---|---|
| temiz 320 s uçuş | - | hiçbir sağlık mesajı |
| GNSS yok 60-80 s | 62,10 s | `GNSS: dead reckoning` |
| | 66,05 s | `Failsafe: position lost (HOLD)` (failsafe'in kendi sözü) |
| | 67,10 s | `GNSS: no position` |
| | 83,00 s | `GNSS: recovered` (fix 80 s'de döndü, 3 s sakinlik) |
| GCS susuyor 50-70 s, `FS_TIMEOUT` 3 s | 53,05 s | `Failsafe: GCS link lost (HOLD)` |
| | 54,10 s | `GCS link: link lost` |
| | 75,05 s | `GCS link: recovered` |
| GCS susuyor 50-58 s, `FS_TIMEOUT` 10 s | 53,10 s | `GCS link: silent 3 s` (UYARI, failsafe yok, tekne AUTO'da kaldı) |
| | 61,10 s | `GCS link: recovered` |

"GCS susuyor 50-70 s" satırı ilginç: varsayılan 3 s `FS_TIMEOUT` ile "sessiz" uyarısı **hiç görünmez** (2 s eşik
+ 1 s debounce = failsafe'in tetiklendiği an), tablo doğrudan HATA'ya geçer. Bu bilerek böyle; uyarı basamağı `FS_TIMEOUT` > 3 s
iken ya da failsafe kapalıyken anlamlı. İki yol da test edildi.
**Simülasyonla bulunan hatalar:**
1. *Failsafe "cleared" diyordu ama neden sürüyordu* (Faz 20'nin mesajı). GNSS kesintisinde failsafe tekneyi HOLD'a
   zorlayınca konum/yön kuralları (yalnızca otonom modda değerlendirilir) devre dışı kalıyor ve kaynak 0,05 s sonra
   `Failsafe cleared: position lost` olarak "temizleniyordu", GNSS hâlâ yokken. Operatör konumun geri geldiğini
   sanırdı. Artık yalnızca neden **gerçekten** kalkınca (ve 2 s kalkık kalınca) "cleared" denir; mod otonomiden
   çıktığı ya da kural kapatıldığı için düşen kaynak sessizce bırakılır (davranış aynı, mesaj değişti).
2. *STM telemetri tazeliği duvar saatiyle ölçülüyordu* (`time.time()` ile damgalı `last_update`): GCS yaşıyla aynı hata
   sınıfı. Pi'de RTC yok; açılışta NTP/GNSS saat senkronu saati yıllarca sıçratırsa STM bir an ölü görünür,
   kumandanın mod anahtarı bir döngü yok sayılırdı. Artık `time.monotonic()`; saatin sıçradığı testle sabit.
3. `SYS_STATUS` bitleri hiç gerçek değildi (0, 0, 0).
**Testler:** her parça için tek hata (seviye + kelimeler), sınırdaki değer hâlâ tamam, debounce **literal saniyelerle**
(1 s, 3 s, 5 s, 10 s), titreyen sensör ve iki duyuru arası ≥ 5 s, tablo-duyuru ayrımı, açılış, IMU kare hızı (yeni
pencere, sayaç yeniden başlaması), STM yeniden başlaması (açılıştan önceki sayılmaz), batarya histerezisi (alçak
ve kritik), her duyuru 50 bayta sığıyor (aşırı sayılarla), SYS_STATUS (parça başına bit, var/etkin/sağlıklı
kuralları, pymavlink ile kodla-çöz), döngü yükü; araçta: parçalar, anlık görüntü birimleri (None ≠ 0, bayat STM →
RC bilinmiyor), her kaynağın operatöre ulaşması, olay günlüğü, `meta.json`, `SYS_STATUS` mesajı, döngünün kendi işini
ölçmesi, duvar saati sıçraması; simülatörde yukarıdaki üç uçuş.
**Mutasyon:** 50 kasıtlı bozma (31 izleyici / döngü zamanlayıcı / SYS_STATUS, 19 bağlama). İlk turda 46'sı yakalandı,
**4'ü sağ kaldı**, hepsi benim testimin zayıflığıydı: (a) kurtulma debounce'u ve (b) yükselme debounce'u, çünkü
testler süreyi modülün kendi sabitiyle yazıyordu (sabit 0 yapılınca test de onunla kayıyordu; yalnızca simülatör
yakalıyordu), (c) kritik bataryanın histerezisi test edilmemişti, (d) negatif meşguliyet süresi ve aynı anda iki
kayıt (sıfıra bölme) testleri birbirini örtüyordu. Literal saniyeli ve doğrudan testler eklendi; 50'si de yakalanıyor.
**Faz 20'ye geri düzeltme için ayrıca:** `cleared` mesajı değişikliği için bozma (sessiz bırakma yerine eski
`_clear`) hem izleyici hem simülatör testleri tarafından yakalanıyor.
**Kalan:**
- `MAV_SYS_STATUS_PREARM_CHECK` biti gönderilmiyor (silahlanma denetimi tek bir fonksiyonda toplanmadı).
- Tablo silahlanmayı engellemiyor; yalnızca raporluyor.
- BAHR-GCS tabloyu göstermiyor (yalnızca `STATUSTEXT` satırlarını görür); GCS'ye bir "sağlık" sayfası eklemek
  ayrı, GCS tarafı bir iş.
- IMU kare hızı eşiği (40 Hz) ve döngü eşikleri gerçek UART ve gerçek Pi 4 ile ölçülmedi.
- İzlenmeyenler: zaman senkronu artığı (`StmClock` dışarı vermiyor), RTCM düzeltme yaşı, GNSS doğruluğu (σ) uyarısı,
  Pi sıcaklığı / CPU kısma, SD kart boş alanı.
