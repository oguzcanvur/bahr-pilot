#include "arming.h"

void Arming_Init(Arming *a, uint32_t now_ms, uint8_t checks, uint32_t boot_grace_ms)
{
    a->state = ARM_STATE_BOOT;
    a->block = ARM_BLOCK_NO_RC;
    a->refusal = ARM_BLOCK_NONE;
    a->checks = checks;
    a->boot_grace_ms = boot_grace_ms;
    a->boot_start_ms = now_ms;
    a->rc_lost_since_ms = now_ms;
    a->rc_was_up = false;
    a->switch_seen_off = false;
    a->prev_switch_on = false;
}

bool Arming_IsArmed(const Arming *a)
{
    return a->state == ARM_STATE_ARMED;
}

/* First reason an arm attempt would be refused right now, or NONE. The
 * order is the order the operator should fix things in. */
static ArmBlock FirstBlock(const Arming *a, const ArmInputs *in)
{
    if (!in->rc_link_up) {
        return ARM_BLOCK_NO_RC;
    }
    if ((in->now_ms - a->boot_start_ms) < a->boot_grace_ms) {
        return ARM_BLOCK_BOOT_GRACE;
    }
    if (!a->switch_seen_off) {
        return (a->refusal != ARM_BLOCK_NONE) ? a->refusal : ARM_BLOCK_SWITCH_ON;
    }
    if ((a->checks & ARM_CHECK_THROTTLE) != 0U && !in->throttle_centered) {
        return ARM_BLOCK_THROTTLE;
    }
    if ((a->checks & ARM_CHECK_IMU) != 0U && !in->imu_ok) {
        return ARM_BLOCK_IMU;
    }
    if ((a->checks & ARM_CHECK_BATTERY) != 0U && !in->battery_ok) {
        return ARM_BLOCK_BATTERY;
    }
    return ARM_BLOCK_NONE;
}

void Arming_Update(Arming *a, const ArmInputs *in)
{
    bool switch_edge_on = in->switch_on && !a->prev_switch_on;
    a->prev_switch_on = in->switch_on;

    /* RC link bookkeeping. Losing RC means the switch position can no
     * longer be trusted, so "seen off" is forgotten until RC is back and
     * the switch is seen off again. */
    if (in->rc_link_up) {
        a->rc_was_up = true;
    } else {
        if (a->rc_was_up) {
            a->rc_lost_since_ms = in->now_ms;
        }
        a->rc_was_up = false;
        a->switch_seen_off = false;
        a->refusal = ARM_BLOCK_NONE;
    }
    if (in->rc_link_up && !in->switch_on) {
        a->switch_seen_off = true;
        a->refusal = ARM_BLOCK_NONE;
    }

    if (a->state == ARM_STATE_ARMED) {
        if (!in->rc_link_up) {
            if ((in->now_ms - a->rc_lost_since_ms) > ARM_RC_LOSS_DISARM_MS) {
                /* long loss: start over, including the boot grace */
                a->state = ARM_STATE_BOOT;
                a->boot_start_ms = in->now_ms;
                a->block = ARM_BLOCK_NO_RC;
            }
            return; /* short dropout: stay armed, the failsafe table stops the motors */
        }
        if (!in->switch_on) {
            a->state = ARM_STATE_DISARMED;
            a->block = ARM_BLOCK_NONE;
            /* fall through to recompute READY below */
        } else {
            a->block = ARM_BLOCK_NONE;
            return;
        }
    }

    ArmBlock block = FirstBlock(a, in);

    if (a->state == ARM_STATE_BOOT && block != ARM_BLOCK_NO_RC && block != ARM_BLOCK_BOOT_GRACE) {
        a->state = ARM_STATE_DISARMED;
    }
    if (a->state == ARM_STATE_BOOT) {
        if (switch_edge_on) {
            /* flipped on during the grace period: that edge is spent, the
             * switch has to be cycled again afterwards */
            a->switch_seen_off = false;
        }
        a->block = block;
        return;
    }

    if (switch_edge_on) {
        if (block == ARM_BLOCK_NONE) {
            a->state = ARM_STATE_ARMED;
            a->block = ARM_BLOCK_NONE;
            return;
        }
        /* refused: the switch must be cycled again, whatever the cause, and
         * the operator keeps seeing the real reason until they do */
        a->switch_seen_off = false;
        a->refusal = block;
    }

    if (block == ARM_BLOCK_NONE) {
        a->state = ARM_STATE_READY;
    } else {
        a->state = ARM_STATE_DISARMED;
    }
    a->block = block;
}
