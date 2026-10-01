#ifndef FAILSAFE_H
#define FAILSAFE_H

/* Implements the Nucleo failsafe priority table decided 2026-09-14
 * (ROADMAP.md section 5), extended 2026-10-01 with a dedicated hardware
 * arm switch, and again with configurable throttle+steering RC mixing
 * (both user's explicit requests — see pi_link.h for the config protocol
 * BAHR-GCS's existing RCMAP_ROLL/RCMAP_THROTTLE/RCMAP_ARM/RCMAP_OVERRIDE
 * parameters and RadioPage calibration screen now actually reach).
 *
 * Highest to lowest priority (higher overrides lower):
 *   0. RC arm switch off             -> stop, overrides everything below —
 *                                       this is the one that's supposed to
 *                                       work even if the Pi and all its
 *                                       software have completely died
 *   1. RC manual-override switch on  -> RC drives the motors directly,
 *                                       via throttle+steering mixing
 *                                       (pi_link.h's RcMapConfig)
 *   2. RC signal lost                -> stop, regardless of anything else
 *   3. RC ok, override off, Pi stale -> stop
 *   4. boot / no command ever        -> stop (the default state until
 *                                       either RC or a fresh Pi command
 *                                       proves otherwise)
 *   5. own firmware stuck            -> TODO: IWDG reset, not implemented
 */

/* Call once after ESC_Init(), SBUS_Init() and PiLink_Init(). */
void Failsafe_Init(void);

/* Call periodically (the project runs this at 100 Hz from the main loop).
 * Reads RC + Pi link state and drives the ESC outputs accordingly; also
 * sends the Nucleo->Pi RC telemetry frame on its own slower schedule
 * (see RC_LINK_TELEMETRY_PERIOD_MS in pi_link.h). */
void Failsafe_Update(void);

#endif /* FAILSAFE_H */
