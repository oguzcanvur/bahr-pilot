#ifndef MODE_SWITCH_H
#define MODE_SWITCH_H

#include <stdint.h>
#include <stdbool.h>

/* Pure logic (no HAL, no globals) so it can be compiled and tested on a
 * host PC — see firmware/reflex/tests/test_mode_switch.c.
 *
 * ArduPilot-style mode switch: one RC channel is split into six PWM bands
 * and BAHR-GCS's MODE1..MODE6 parameters say which flight mode each band
 * selects. The STM only needs to know two things about it: which band the
 * switch is in (reported to the Pi, which owns the mode table), and
 * whether that band is MANUAL (the STM then drives the motors from the
 * sticks itself, with no Pi involved). */

#define MODE_SWITCH_SLOTS 6U

/* Band edges in microseconds, identical to ArduPilot's
 * RC_Channel::read_mode_switch(): <=1230 -> slot 1, <=1360 -> 2,
 * <=1490 -> 3, <=1620 -> 4, <=1749 -> 5, else 6. */

/* Slot 1..6 for a pulse width in microseconds; 0 if the value is outside
 * 901..2199 (ArduPilot treats that as "no valid signal"). */
uint8_t ModeSwitch_SlotFromUs(uint16_t pulse_us);

/* Maps a raw SBUS value through that channel's calibrated min/max to a
 * 1000..2000 us equivalent. min >= max (uncalibrated or corrupt) gives
 * 0, which ModeSwitch_SlotFromUs() reports as invalid. */
uint16_t ModeSwitch_RawToUs(uint16_t raw, uint16_t min, uint16_t max);

/* True if bit (slot-1) of manual_mask is set. Slot 0 (invalid) is never
 * manual: a missing signal must not hand the sticks to anyone. */
bool ModeSwitch_SlotIsManual(uint8_t slot, uint8_t manual_mask);

#endif /* MODE_SWITCH_H */
