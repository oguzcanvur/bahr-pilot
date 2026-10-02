"""Rover-style flight-mode numbers, matching sim/fake_vehicle.py and what
BAHR-GCS (configured for a MAV_AUTOPILOT_ARDUPILOTMEGA rover) expects."""

MODE_MANUAL = 0
MODE_HOLD = 4
MODE_LOITER = 5
MODE_AUTO = 10
MODE_RTL = 11
MODE_GUIDED = 15

MODE_NAMES = {
    MODE_MANUAL: "MANUAL",
    MODE_HOLD: "HOLD",
    MODE_LOITER: "LOITER",
    MODE_AUTO: "AUTO",
    MODE_RTL: "RTL",
    MODE_GUIDED: "GUIDED",
}
