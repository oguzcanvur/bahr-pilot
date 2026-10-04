#ifndef ARMING_H
#define ARMING_H

#include <stdint.h>
#include <stdbool.h>

/* Arm state machine. Pure logic (no HAL, no globals) so it is compiled and
 * tested on a host PC — see tests/c/test_arming.c.
 *
 * Until 2026-10-02 "armed" was simply "the arm switch reads on", re-evaluated
 * every tick. Power the boat up with the switch already on and the first
 * valid SBUS frame armed it instantly — motors could spin the moment the
 * receiver came alive. This machine closes that and a few related holes:
 *
 *   BOOT      power-up (or RC came back after a long loss). Motors held at
 *             neutral for BOOT_GRACE_MS and until a valid RC link exists.
 *   DISARMED  RC is fine but the switch has not been seen OFF yet, or a
 *             pre-arm check is failing.
 *   READY     switch seen OFF and every enabled check passes; flipping the
 *             switch ON will arm.
 *   ARMED     motors are allowed to move (what happens inside ARMED is the
 *             failsafe priority table's job, not this module's).
 *
 * Arming only happens on an OFF -> ON edge while READY. If a check fails at
 * that edge the request is refused, the reason is reported, and the switch
 * must be cycled again — a stick nudged off-centre at the wrong moment can
 * never "arm later by itself". */

typedef enum {
    ARM_STATE_BOOT = 0,
    ARM_STATE_DISARMED = 1,
    ARM_STATE_READY = 2,
    ARM_STATE_ARMED = 3,
} ArmState;

/* Why arming is not (yet) possible — reported to the Pi, which turns it
 * into a "PreArm: ..." message for BAHR-GCS. Highest priority first. */
typedef enum {
    ARM_BLOCK_NONE = 0,
    ARM_BLOCK_NO_RC = 1,       /* no valid RC link */
    ARM_BLOCK_BOOT_GRACE = 2,  /* still inside the power-up grace period */
    ARM_BLOCK_SWITCH_ON = 3,   /* arm switch is on but was never seen off: flip it off first */
    ARM_BLOCK_THROTTLE = 4,    /* throttle stick not centred */
    ARM_BLOCK_IMU = 5,         /* IMU check enabled and IMU not valid */
    ARM_BLOCK_BATTERY = 6,     /* battery check enabled and battery not valid / too low */
} ArmBlock;

/* ARMING_CHECK-style bitmask of the optional checks. RC link, boot grace
 * and "switch seen off" are always enforced and cannot be disabled. */
#define ARM_CHECK_THROTTLE 0x01U
#define ARM_CHECK_IMU      0x02U
#define ARM_CHECK_BATTERY  0x04U
/* IMU and battery default OFF: neither has been verified on real hardware
 * (the battery divider ratio is a placeholder), so a wrong reading must not
 * be able to lock the boat out of arming until someone has checked them. */
#define ARM_CHECKS_DEFAULT ARM_CHECK_THROTTLE

#define ARM_BOOT_GRACE_MS_DEFAULT 2000U
/* RC lost for longer than this while armed drops back to BOOT: the boat
 * must be re-armed by hand instead of resuming on its own when the
 * transmitter happens to come back. A shorter dropout keeps ARMED (the
 * failsafe table already stops the motors while RC is down). */
#define ARM_RC_LOSS_DISARM_MS 5000U

typedef struct {
    uint32_t now_ms;
    bool rc_link_up;
    bool switch_on;
    bool throttle_centered;
    bool imu_ok;
    bool battery_ok;
} ArmInputs;

typedef struct {
    ArmState state;
    ArmBlock block;
    ArmBlock refusal; /* why the last arm attempt was refused; sticky until the switch is cycled */
    uint8_t checks;
    uint32_t boot_grace_ms;
    uint32_t boot_start_ms;
    uint32_t rc_lost_since_ms;
    bool rc_was_up;
    bool switch_seen_off;
    bool prev_switch_on;
} Arming;

void Arming_Init(Arming *a, uint32_t now_ms, uint8_t checks, uint32_t boot_grace_ms);
void Arming_Update(Arming *a, const ArmInputs *in);
bool Arming_IsArmed(const Arming *a);

#endif /* ARMING_H */
