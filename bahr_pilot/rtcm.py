"""RTCM3 baytlarını MAVLink GPS_RTCM_DATA mesajlarına bölme.

GPS_RTCM_DATA'nın veri alanı sabit 180 bayt. `flags` baytı: bit0 =
parçalanmış mı, bit1-2 = parça no (0-3), bit3-7 = sıra no (0-31) — bkz.
mesajın kendi açıklaması (pymavlink). 4 parçadan büyük bir NTRIP okuması
yeni bir sıra numarasıyla devam eder.

Bu dosya saf mantık — soket, Qt ya da pymavlink'e bağımlı değil, bu yüzden
donanım/ağ olmadan test edilebilir.

BAHR-GCS'nin (github.com/oguzcanvur/bahr-gcs) `gcs/rtcm.py` dosyasıyla
birebir aynı mantık — bahr_pilot kendi başına klonlanabilsin diye burada
da birebir tutuluyor (çapraz-repo bağımlılık olmasın diye)."""
from __future__ import annotations

MAX_PAYLOAD = 180
MAX_FRAGMENTS = 4


class RtcmFragmenter:
    def __init__(self) -> None:
        self._seq = 0

    def fragment(self, data: bytes) -> list[tuple[int, int, bytes]]:
        """(flags, uzunluk, veri) üçlüleri — her biri tek bir
        gps_rtcm_data_send çağrısına karşılık gelir."""
        if not data:
            return []
        chunks = [data[i:i + MAX_PAYLOAD] for i in range(0, len(data), MAX_PAYLOAD)]
        if len(chunks) == 1:
            return [(0, len(chunks[0]), chunks[0])]

        out: list[tuple[int, int, bytes]] = []
        for group_start in range(0, len(chunks), MAX_FRAGMENTS):
            group = list(chunks[group_start:group_start + MAX_FRAGMENTS])
            # Belirsiz durum: grup 4'ten az parçayla bitiyor ama son parça
            # tam MAX_PAYLOAD bayt — alıcı "daha fazlası geliyor" sanabilir
            # (bkz. RtcmReassembler). Boş (0 bayt) bir sonlandırıcı parça
            # ekleyerek grubu erkenden bitmiş olarak işaretlemesini sağlar.
            if len(group) < MAX_FRAGMENTS and len(group[-1]) == MAX_PAYLOAD:
                group.append(b"")
            seq = self._seq
            self._seq = (self._seq + 1) % 32
            for fragment_id, chunk in enumerate(group):
                flags = 1 | (fragment_id << 1) | (seq << 3)
                out.append((flags, len(chunk), chunk))
        return out


class RtcmReassembler:
    """RtcmFragmenter'in tersi — GPS_RTCM_DATA parçalarını geri RTCM3
    baytlarına birleştirir. Kayıp/sıra dışı bir parça gelirse o anki mesajı
    sessizce atar (bir sonraki fragment_id==0 ile yeniden başlar) — bozuk
    yarım bir RTCM çerçevesini GNSS'e yazmaktansa bir düzeltme paketini
    kaçırmak daha zararsız."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._seq: int | None = None
        self._next_fragment = 0

    def add(self, flags: int, length: int, data: bytes) -> bytes | None:
        payload = bytes(data[:length])
        if not flags & 1:  # parçalanmamış — kendi başına tam bir mesaj
            return payload

        fragment_id = (flags >> 1) & 0b11
        seq = flags >> 3

        if fragment_id == 0:
            self._buf = bytearray(payload)
            self._seq = seq
            self._next_fragment = 1
        elif seq != self._seq or fragment_id != self._next_fragment:
            self._reset()
            return None
        else:
            self._buf += payload
            self._next_fragment += 1

        if length < MAX_PAYLOAD or self._next_fragment >= MAX_FRAGMENTS:
            out = bytes(self._buf)
            self._reset()
            return out
        return None

    def _reset(self) -> None:
        self._buf = bytearray()
        self._seq = None
        self._next_fragment = 0
