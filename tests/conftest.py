"""Shared fixtures: build the firmware's host-testable C with whatever host
C compiler is available, once per test session.

Compiler lookup: $BAHR_CC (e.g. "gcc" or "python -m ziglang cc"), else gcc/
cc/clang on PATH, else the `ziglang` pip package, else every test that needs
`build` skips. The ARM firmware build itself (arm-none-eabi-gcc) is a
separate check.
"""
from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parent.parent
FIRMWARE_INC = REPO / "firmware" / "reflex" / "Core" / "Inc"
FIRMWARE_SRC = REPO / "firmware" / "reflex" / "Core" / "Src"
C_TESTS = REPO / "tests" / "c"


def _find_compiler() -> list[str] | None:
    override = os.environ.get("BAHR_CC")
    if override:
        return shlex.split(override)
    for name in ("gcc", "cc", "clang"):
        path = shutil.which(name)
        if path:
            return [path]
    try:
        import ziglang  # noqa: F401
    except ImportError:
        return None
    return [sys.executable, "-m", "ziglang", "cc"]


@pytest.fixture(scope="session")
def build(tmp_path_factory):
    compiler = _find_compiler()
    if compiler is None:
        pytest.skip("no host C compiler (set BAHR_CC, or install gcc / pip install ziglang)")
    out = tmp_path_factory.mktemp("cbuild")

    def compile_exe(name, sources, *includes):
        exe = out / f"{name}.exe"
        cmd = [*compiler, "-Wall", "-Wextra"]
        for inc in includes:
            cmd += ["-I", str(inc)]
        cmd += [str(s) for s in sources] + ["-lm", "-o", str(exe)]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        assert result.returncode == 0, f"compile failed:\n{result.stderr}"
        return exe

    src = FIRMWARE_SRC
    return SimpleNamespace(
        mode_switch=compile_exe(
            "test_mode_switch", [C_TESTS / "test_mode_switch.c", src / "mode_switch.c"],
            FIRMWARE_INC),
        arming=compile_exe(
            "test_arming", [C_TESTS / "test_arming.c", src / "arming.c"], FIRMWARE_INC),
        motor=compile_exe(
            "test_motor", [C_TESTS / "test_motor.c", src / "motor.c"], FIRMWARE_INC),
        clock_core=compile_exe(
            "test_clock_core", [C_TESTS / "test_clock_core.c", src / "clock_core.c"], FIRMWARE_INC),
        imu_reports=compile_exe(
            "test_imu_reports", [C_TESTS / "test_imu_reports.c", src / "imu_reports.c"],
            FIRMWARE_INC),
        pi_link=compile_exe(
            "pi_link_harness", [C_TESTS / "pi_link_harness.c", src / "pi_link.c"],
            C_TESTS / "stubs", FIRMWARE_INC),
        failsafe=compile_exe(
            "failsafe_harness",
            [C_TESTS / "failsafe_harness.c", src / "failsafe.c", src / "arming.c",
             src / "motor.c", src / "mode_switch.c"],
            C_TESTS / "stubs", FIRMWARE_INC),
    )
