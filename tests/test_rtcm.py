"""Fragment/reassemble round-trip tests for bahr_pilot/rtcm.py — the
module vendored from gcs/rtcm.py so bahr_pilot has no cross-repo import
(see bahr_pilot/rtcm.py's own docstring). Includes the boundary case that
was a real bug when this logic was first tested on 2026-09-15 (ROADMAP.md
section 4): data landing on an exact multiple of 180 bytes and ending with
fewer than 4 fragments used to silently drop the last fragment.
"""
from __future__ import annotations

import os
import random

from bahr_pilot.rtcm import MAX_PAYLOAD, RtcmFragmenter, RtcmReassembler


def _round_trip(data: bytes) -> bytes:
    """Each completed group of <=4 fragments (MAVLink's GPS_RTCM_DATA flags
    byte only has room for 4 fragments/32 sequences per group — an
    inherent protocol limit) comes back as its own separate `out`, not one
    combined result for messages spanning multiple groups (>720 bytes).
    The real consumer (rtcm_forward.py) writes each completed group
    straight to the GNSS serial port as it arrives, so concatenating every
    non-None result here is what actually reconstructs the original
    bytes — keeping only the last one (an earlier version of this test
    did) silently drops every group but the final one."""
    fragmenter = RtcmFragmenter()
    reassembler = RtcmReassembler()
    out_parts = []
    for flags, length, chunk in fragmenter.fragment(data):
        out = reassembler.add(flags, length, chunk)
        if out is not None:
            out_parts.append(out)
    return b"".join(out_parts)


def test_empty_input():
    assert RtcmFragmenter().fragment(b"") == []


def test_single_small_message():
    data = os.urandom(50)
    assert _round_trip(data) == data


def test_exact_one_payload():
    data = os.urandom(MAX_PAYLOAD)
    assert _round_trip(data) == data


def test_multi_fragment_message():
    data = os.urandom(MAX_PAYLOAD * 3 + 40)
    assert _round_trip(data) == data


def test_exact_multiple_of_payload_boundary_case():
    """The bug found 2026-09-15: data landing on an exact multiple of
    MAX_PAYLOAD (so the last real chunk is a full chunk, not a short one)
    used to leave the reassembler waiting for a 4th fragment that never
    came. The fix adds an empty terminator fragment."""
    for n_chunks in (1, 2, 3):
        data = os.urandom(MAX_PAYLOAD * n_chunks)
        assert _round_trip(data) == data


def test_many_random_sizes():
    rng = random.Random(1234)
    for _ in range(300):
        size = rng.randint(1, MAX_PAYLOAD * 5)
        data = os.urandom(size)
        assert _round_trip(data) == data


def test_lost_fragment_drops_message_not_crash():
    data = os.urandom(MAX_PAYLOAD * 3)
    fragmenter = RtcmFragmenter()
    fragments = fragmenter.fragment(data)
    reassembler = RtcmReassembler()
    # Drop the second fragment entirely — simulates a lost UDP/radio packet.
    results = [reassembler.add(flags, length, chunk)
               for i, (flags, length, chunk) in enumerate(fragments) if i != 1]
    assert all(r is None for r in results)  # never completes, never raises
