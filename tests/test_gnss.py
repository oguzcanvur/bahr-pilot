"""Phase 5: GNSS parsing, quality classes, validity/staleness and the merge of
the RTD100 and the echoMAP.

Sentences are built from the NovAtel field layout the readers assume — see
bahr_pilot/gnss.py's docstring: they prove consistency with that layout, not
that the layout is Bynav's. (No real GNSS sentences with real values were ever
captured: the one RTD100 capture was indoors, all fields empty.)
"""
from __future__ import annotations

import pytest

from bahr_pilot.gnss import (
    GnssQuality, classify_pos_type, fix_is_usable, heading_is_usable, mav_fix_type, parse_bestposa,
    parse_gga, parse_gsa, parse_headinga,
)
from bahr_pilot.sensors import GnssState, _crc32_novatel, read_bynav_logs, read_rtd100
from bahr_pilot.state import VehicleState


def bestposa(sol="SOL_COMPUTED", pos_type="NARROW_INT", lat="41.0", lon="29.0", hgt="12.5",
             lat_s="0.008", lon_s="0.006", hgt_s="0.012", diff_age="1.0", sats="24") -> list[str]:
    return [sol, pos_type, lat, lon, hgt, "35.0", "WGS84", lat_s, lon_s, hgt_s, '"0"', diff_age,
            "0.0", sats, "22", "22", "22", "0", "0", "0", "0"]


def headinga(sol="SOL_COMPUTED", pos_type="NARROW_INT", heading="153.2", sigma="0.3") -> list[str]:
    return [sol, pos_type, "1.05", heading, "0.5", "0.0", sigma, "0.6", '"0"', "20", "18", "18", "18"]


def gga(quality="1", lat="4100.0000", ns="N", lon="02900.0000", ew="E", sats="12", hdop="0.9",
        alt="12.5") -> list[str]:
    return ["120000.00", lat, ns, lon, ew, quality, sats, hdop, alt, "M", "35.0", "M", "", ""]


def gsa(mode="3", hdop="0.9", vdop="1.4") -> list[str]:
    return ["A", mode] + ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "", ""] + \
           ["1.6", hdop, vdop]


# -- classification -----------------------------------------------------------

@pytest.mark.parametrize("pos_type,expected", [
    ("NONE", GnssQuality.NO_FIX), ("INSUFFICIENT_OBS", GnssQuality.NO_FIX),
    ("FIXEDPOS", GnssQuality.NO_FIX), ("FIXEDHEIGHT", GnssQuality.NO_FIX),
    ("SINGLE", GnssQuality.FIX_3D),
    ("PSRDIFF", GnssQuality.DGPS), ("WAAS", GnssQuality.DGPS), ("SBAS", GnssQuality.DGPS),
    ("PPP", GnssQuality.DGPS), ("PPP_CONVERGING", GnssQuality.DGPS),
    ("L1_FLOAT", GnssQuality.FLOAT), ("NARROW_FLOAT", GnssQuality.FLOAT),
    ("IONOFREE_FLOAT", GnssQuality.FLOAT), ("INS_RTKFLOAT", GnssQuality.FLOAT),
    ("L1_INT", GnssQuality.RTK_FIXED), ("NARROW_INT", GnssQuality.RTK_FIXED),
    ("WIDE_INT", GnssQuality.RTK_FIXED), ("INS_RTKFIXED", GnssQuality.RTK_FIXED),
    ("SOMETHING_NEW", GnssQuality.FIX_3D),    # unknown: under-claim, never RTK
])
def test_position_type_classification(pos_type, expected):
    assert classify_pos_type(pos_type) == expected


def test_quality_ordering_and_mavlink_mapping():
    assert GnssQuality.NO_FIX < GnssQuality.FIX_2D < GnssQuality.FIX_3D < GnssQuality.DGPS \
        < GnssQuality.FLOAT < GnssQuality.RTK_FIXED
    assert [mav_fix_type(q) for q in GnssQuality] == [1, 2, 3, 4, 5, 6]


# -- parsing --------------------------------------------------------------------

def test_bestposa_rtk_fixed():
    fix = parse_bestposa(bestposa(), t=10.0)
    assert fix.quality == GnssQuality.RTK_FIXED and fix.source == "rtd100"
    assert (fix.lat, fix.lon, fix.alt_m) == (41.0, 29.0, 12.5)
    assert fix.h_acc_m == pytest.approx(0.01)       # hypot(0.008, 0.006)
    assert fix.v_acc_m == pytest.approx(0.012)
    assert fix.diff_age_s == 1.0 and fix.satellites == 24 and fix.t == 10.0


@pytest.mark.parametrize("fields", [
    bestposa(sol="INSUFFICIENT_OBS", pos_type="NONE"),
    bestposa(sol="NO_CONVERGENCE"),
    bestposa(sol="COLD_START"),
    bestposa(pos_type="NONE"),
    bestposa(pos_type="FIXEDPOS"),        # receiver was TOLD its position, nothing measured
    bestposa(lat=""), bestposa(lon="abc"), bestposa(lat="nan"),
    ["SOL_COMPUTED", "SINGLE"],           # truncated
    [],
])
def test_bestposa_that_is_not_a_measured_fix_is_rejected(fields):
    assert parse_bestposa(fields, t=1.0) is None


def test_bestposa_with_missing_optional_fields_still_parses():
    fix = parse_bestposa(["SOL_COMPUTED", "SINGLE", "41.0", "29.0"], t=1.0)
    assert fix.quality == GnssQuality.FIX_3D
    assert fix.h_acc_m is None and fix.satellites == 0   # unknown stays unknown, not 0.0 m


def test_headinga():
    h = parse_headinga(headinga(), t=3.0)
    assert h.heading_deg == 153.2 and h.acc_deg == 0.3 and h.baseline_m == 1.05
    assert h.quality == GnssQuality.RTK_FIXED
    assert parse_headinga(headinga(heading="360.0"), t=3.0).heading_deg == 0.0


@pytest.mark.parametrize("fields", [
    headinga(sol="INSUFFICIENT_OBS", pos_type="NONE"), headinga(pos_type="NONE"),
    headinga(heading=""), headinga(heading="x"), ["SOL_COMPUTED"],
])
def test_headinga_without_a_solution_is_rejected(fields):
    assert parse_headinga(fields, t=1.0) is None


@pytest.mark.parametrize("code,expected", [
    ("1", GnssQuality.FIX_3D), ("2", GnssQuality.DGPS), ("4", GnssQuality.RTK_FIXED),
    ("5", GnssQuality.FLOAT),
])
def test_gga_quality(code, expected):
    fix = parse_gga(gga(quality=code), t=1.0, source="echomap")
    assert fix.quality == expected and fix.hdop == 0.9 and fix.satellites == 12


@pytest.mark.parametrize("code", ["0", "6", "7", ""])
def test_gga_without_a_real_fix_is_rejected(code):
    assert parse_gga(gga(quality=code), t=1.0, source="echomap") is None


def test_gga_coordinates_and_hemispheres():
    fix = parse_gga(gga(lat="4030.0000", ns="S", lon="07400.0000", ew="W"), t=1.0, source="echomap")
    assert fix.lat == pytest.approx(-40.5) and fix.lon == pytest.approx(-74.0)


def test_gsa():
    d = parse_gsa(gsa(mode="2", hdop="2.5", vdop="3.1"))
    assert (d.mode, d.hdop, d.vdop) == (2, 2.5, 3.1)
    assert parse_gsa(["A", "3"]) is None


# -- validity -------------------------------------------------------------------

def test_fix_validity_rules():
    good = parse_bestposa(bestposa(), t=100.0)
    assert fix_is_usable(good, now=100.5)
    assert not fix_is_usable(good, now=101.5)                       # RTD100 fix older than 1 s
    assert not fix_is_usable(None, now=100.0)
    sloppy = parse_bestposa(bestposa(lat_s="9", lon_s="9"), t=100.0)  # 12.7 m sigma
    assert not fix_is_usable(sloppy, now=100.1)
    null_island = parse_bestposa(bestposa(lat="0.0", lon="0.0"), t=100.0)
    assert not fix_is_usable(null_island, now=100.1)


def test_echomap_fix_is_allowed_to_be_older_than_an_rtd100_fix():
    fix = parse_gga(gga(), t=100.0, source="echomap")              # 1 Hz sentence
    assert fix_is_usable(fix, now=102.0)
    assert not fix_is_usable(fix, now=103.0)


def test_heading_validity_rules():
    h = parse_headinga(headinga(), t=50.0)
    assert heading_is_usable(h, now=54.0)
    assert not heading_is_usable(h, now=56.0)
    assert not heading_is_usable(parse_headinga(headinga(sigma="30"), t=50.0), now=50.1)
    assert not heading_is_usable(None, now=0.0)


# -- merging: one shared state ---------------------------------------------------

def feed_rtd100(gnss, now, **kw):
    gnss.apply_rtd100("BESTPOSA", bestposa(**kw.get("pos", {})), now=now)
    gnss.apply_rtd100("HEADINGA", headinga(**kw.get("head", {})), now=now)


def feed_echomap(gnss, now):
    # a few metres from the RTD100's position, and a MAGNETIC heading 6 deg off
    gnss.apply_echomap("GPGGA", gga(lat="4100.0300", lon="02900.0300"), now=now)
    gnss.apply_echomap("HCHDM", ["147.0", "M"], now=now)


def test_rtd100_wins_and_the_echomap_cannot_overwrite_it():
    """Regression: the vehicle used to give each reader its own state, so the
    echoMAP's weaker GPS and magnetic heading overwrote the RTD100's RTK
    position and true heading every second (fix type flapping 6 <-> 3)."""
    gnss, vehicle = GnssState(), VehicleState()
    for step in range(10):
        now = 100.0 + step * 0.2
        feed_rtd100(gnss, now)
        gnss.publish(vehicle, now)
        feed_echomap(gnss, now + 0.05)
        gnss.publish(vehicle, now + 0.05)
        assert (vehicle.lat, vehicle.lon) == (41.0, 29.0)
        assert vehicle.heading_deg == 153.2 and vehicle.heading_valid
        assert vehicle.fix_type == 6 and vehicle.gnss_source == "rtd100"


def test_falls_back_to_the_echomap_when_the_rtd100_goes_quiet():
    gnss, vehicle = GnssState(), VehicleState()
    feed_rtd100(gnss, 100.0)
    feed_echomap(gnss, 100.0)
    gnss.publish(vehicle, 100.5)
    assert vehicle.gnss_source == "rtd100"

    gnss.publish(vehicle, 101.6)                       # RTD100 fix now 1.6 s old
    assert vehicle.gnss_source == "echomap"
    assert vehicle.lat == pytest.approx(41.0005) and vehicle.fix_type == 3
    assert vehicle.hdop == 0.9                          # GGA's own HDOP
    # the 1 Hz HEADINGA is only 1.6 s old, well inside its 5 s allowance: still good
    assert vehicle.heading_valid and vehicle.heading_deg == 153.2

    gnss.publish(vehicle, 105.5)                        # now it is stale too
    assert not vehicle.heading_valid                    # the echoMAP has no true heading to offer
    assert vehicle.heading_deg == 153.2                 # last value kept for display only


def test_magnetic_heading_is_never_published_as_the_vehicle_heading():
    gnss, vehicle = GnssState(), VehicleState()
    feed_echomap(gnss, 10.0)
    gnss.publish(vehicle, 10.1)
    assert gnss.magnetic_heading_deg == 147.0
    assert not vehicle.heading_valid and vehicle.heading_deg == 0.0


def test_everything_stale_means_no_position_and_no_heading():
    gnss, vehicle = GnssState(), VehicleState()
    feed_rtd100(gnss, 100.0)
    feed_echomap(gnss, 100.0)
    gnss.publish(vehicle, 100.1)
    assert vehicle.lat is not None
    gnss.publish(vehicle, 130.0)                        # cable pulled, 30 s later
    assert vehicle.lat is None and vehicle.lon is None
    assert vehicle.fix_type == 1 and vehicle.gnss_quality == 0 and vehicle.gnss_source == ""
    assert vehicle.satellites == 0 and vehicle.h_acc_m is None
    assert not vehicle.heading_valid


def test_a_lost_solution_is_not_remembered():
    gnss, vehicle = GnssState(), VehicleState()
    feed_rtd100(gnss, 100.0)
    gnss.apply_rtd100("BESTPOSA", bestposa(sol="INSUFFICIENT_OBS", pos_type="NONE"), now=100.1)
    gnss.publish(vehicle, 100.2)
    assert vehicle.lat is None and vehicle.fix_type == 1


def test_heading_expires_on_its_own_clock():
    gnss, vehicle = GnssState(), VehicleState()
    feed_rtd100(gnss, 100.0)
    for t in (100.1, 100.4):                              # position keeps coming at 5 Hz ...
        gnss.apply_rtd100("BESTPOSA", bestposa(), now=t)
    gnss.publish(vehicle, 100.5)
    assert vehicle.heading_valid
    for t in (101.0, 102.0, 103.0, 104.0, 105.0, 105.9):
        gnss.apply_rtd100("BESTPOSA", bestposa(), now=t)
    gnss.publish(vehicle, 106.0)                          # ... but HEADINGA stopped 6 s ago
    assert vehicle.lat is not None and not vehicle.heading_valid


def test_2d_gsa_mode_downgrades_a_3d_fix_but_not_rtk():
    gnss = GnssState()
    gnss.apply_rtd100("BESTPOSA", bestposa(pos_type="SINGLE", lat_s="1", lon_s="1"), now=10.0)
    gnss.apply_rtd100("GPGSA", gsa(mode="2", hdop="3.0", vdop="9.9"), now=10.0)
    fix = gnss.best_fix(now=10.1)
    assert fix.quality == GnssQuality.FIX_2D and (fix.hdop, fix.vdop) == (3.0, 9.9)

    gnss.apply_rtd100("BESTPOSA", bestposa(pos_type="NARROW_INT"), now=10.2)
    assert gnss.best_fix(now=10.3).quality == GnssQuality.RTK_FIXED


def test_dops_are_attached_only_while_fresh():
    gnss = GnssState()
    gnss.apply_rtd100("BESTPOSA", bestposa(), now=10.0)
    gnss.apply_rtd100("GPGSA", gsa(hdop="1.1", vdop="1.7"), now=10.0)
    assert gnss.best_fix(now=10.5).hdop == 1.1
    gnss.apply_rtd100("BESTPOSA", bestposa(), now=14.0)
    assert gnss.best_fix(now=14.5).hdop is None            # GSA is 4.5 s old: not trusted


def test_hdop_from_gga_on_the_rtd100_port():
    gnss = GnssState()
    gnss.apply_rtd100("BESTPOSA", bestposa(), now=10.0)
    gnss.apply_rtd100("GPGGA", gga(hdop="0.8"), now=10.0)
    assert gnss.best_fix(now=10.1).hdop == 0.8


def test_depth_and_water_temperature_expire():
    gnss, vehicle = GnssState(), VehicleState()
    gnss.apply_echomap("SDDPT", ["7.5", "0.0"], now=10.0)
    gnss.apply_echomap("SDMTW", ["18.2", "C"], now=10.0)
    gnss.publish(vehicle, 12.0)
    assert (vehicle.depth_m, vehicle.water_temp_c) == (7.5, 18.2)
    gnss.publish(vehicle, 20.0)
    assert vehicle.depth_m is None and vehicle.water_temp_c is None


def test_publishing_keeps_the_arm_gate_working():
    """Arming in AUTO/GUIDED needs fix_type >= 3: a stale or absent fix must
    read as 1, a fresh RTK fix as 6."""
    gnss, vehicle = GnssState(), VehicleState()
    feed_rtd100(gnss, 10.0)
    gnss.publish(vehicle, 10.1)
    assert vehicle.fix_type == 6
    gnss.publish(vehicle, 12.0)
    assert vehicle.fix_type == 1


# -- the readers (wire level) -------------------------------------------------------

def bynav_line(name: str, fields: list[str]) -> bytes:
    body = f"{name},COM1,0,0.0,FINESTEERING,2200,100000.000,00000000,0000,0;" + ",".join(fields)
    return f"#{body}*{_crc32_novatel(body.encode()):08x}\r\n".encode()


def nmea_line(sentence_id: str, fields: list[str]) -> bytes:
    body = sentence_id + "," + ",".join(fields)
    checksum = 0
    for byte in body.encode():
        checksum ^= byte
    return f"${body}*{checksum:02X}\r\n".encode()


class FakePort:
    """Hands out its data in awkward chunks, like a real serial port."""

    def __init__(self, data: bytes, chunk: int = 7) -> None:
        self._data, self._chunk = data, chunk

    def read(self, _n: int) -> bytes:
        if not self._data:
            raise StopIteration
        piece, self._data = self._data[:self._chunk], self._data[self._chunk:]
        return piece


def collect(reader, data: bytes, chunk: int = 7):
    out = []
    try:
        for item in reader(FakePort(data, chunk)):
            out.append(item)
    except (StopIteration, RuntimeError):
        pass
    return out


def test_reader_accepts_good_crc_and_rejects_bad_crc():
    good = bynav_line("BESTPOSA", bestposa())
    bad = good.replace(b"41.0", b"42.0", 1)               # payload altered, CRC now wrong
    got = collect(read_bynav_logs, bad + good)
    assert [kind for kind, _ in got] == ["BESTPOSA"]
    assert got[0][1][2] == "41.0"


def test_reader_sees_both_formats_on_the_rtd100_port_and_checks_both():
    stream = (bynav_line("BESTPOSA", bestposa()) + nmea_line("GPGGA", gga())
              + nmea_line("GPGSA", gsa()) + bynav_line("HEADINGA", headinga())
              + nmea_line("GPGGA", gga()).replace(b"*", b"*00#", 1)       # corrupt checksum
              + b"garbage line\r\n")
    got = collect(read_rtd100, stream)
    assert [kind for kind, _ in got] == ["BESTPOSA", "GPGGA", "GPGSA", "HEADINGA"]


def test_end_to_end_from_wire_bytes_to_vehicle_state():
    stream = (bynav_line("BESTPOSA", bestposa()) + bynav_line("HEADINGA", headinga())
              + nmea_line("GPGSA", gsa(hdop="0.7", vdop="1.2")))
    gnss, vehicle = GnssState(), VehicleState()
    for kind, fields in collect(read_rtd100, stream, chunk=5):
        gnss.apply_rtd100(kind, fields, now=50.0)
    gnss.publish(vehicle, 50.1)
    assert (vehicle.lat, vehicle.lon) == (41.0, 29.0)
    assert vehicle.fix_type == 6 and vehicle.heading_valid and vehicle.heading_deg == 153.2
    assert vehicle.hdop == 0.7 and vehicle.satellites == 24
    assert vehicle.h_acc_m == pytest.approx(0.01)
