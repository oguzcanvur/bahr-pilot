"""Version numbers the autopilot reports, kept apart because they change for
different reasons (docs/ARCHITECTURE.md section 8).

AUTOPILOT_VERSION     SemVer of this package (bahr_pilot.__version__).
BAHR_LINK_VERSION     Pi <-> Nucleo wire protocol (firmware pi_link.h). Bump on
                      ANY change to a frame layout; the two sides must match.
MISSION_FORMAT_VERSION  What the GCS uploads: 1 = flat NAV_WAYPOINT list,
                      slot 0 = home, no line ids (docs/BAHR_GCS_ARCHITECTURE.md).
"""
from bahr_pilot import __version__ as AUTOPILOT_VERSION

# 1: original 7/16-byte frames; 2: mode switch (config 25 B, telemetry 43 B);
# 3: arm state in telemetry (44 B); 4: IMU frame (28 B, 0xE6 0x6E) and the TX ring.
BAHR_LINK_VERSION = 4
MISSION_FORMAT_VERSION = 1


def banner() -> str:
    """One line for STATUSTEXT (50 characters max)."""
    return f"BAHR-Pilot {AUTOPILOT_VERSION} link{BAHR_LINK_VERSION} msn{MISSION_FORMAT_VERSION}"
