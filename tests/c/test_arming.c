/* Host-side test for firmware/reflex/Core/Src/arming.c (pure logic).
 * Built and run by tests/test_c_firmware_logic.py. */
#include <stdio.h>

#include "arming.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            s_failures++;                                                    \
        }                                                                    \
    } while (0)

static Arming s_a;
static uint32_t s_t;

static void start(uint8_t checks, uint32_t t0)
{
    s_t = t0;
    Arming_Init(&s_a, s_t, checks, ARM_BOOT_GRACE_MS_DEFAULT);
}

static void step(bool rc, bool sw, bool throttle_ok, uint32_t advance_ms)
{
    s_t += advance_ms;
    ArmInputs in = {.now_ms = s_t, .rc_link_up = rc, .switch_on = sw,
                    .throttle_centered = throttle_ok, .imu_ok = true, .battery_ok = true};
    Arming_Update(&s_a, &in);
}

static void step_sensors(bool rc, bool sw, bool imu, bool battery, uint32_t advance_ms)
{
    s_t += advance_ms;
    ArmInputs in = {.now_ms = s_t, .rc_link_up = rc, .switch_on = sw,
                    .throttle_centered = true, .imu_ok = imu, .battery_ok = battery};
    Arming_Update(&s_a, &in);
}

/* Boot with RC up and the switch off, wait out the grace period. */
static void reach_ready(void)
{
    step(true, false, true, 10);
    step(true, false, true, ARM_BOOT_GRACE_MS_DEFAULT);
}

static void test_boot_without_rc(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    step(false, false, true, 5000);
    CHECK(s_a.state == ARM_STATE_BOOT);
    CHECK(s_a.block == ARM_BLOCK_NO_RC);
    CHECK(!Arming_IsArmed(&s_a));
}

static void test_boot_grace_then_ready(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    step(true, false, true, 100);
    CHECK(s_a.state == ARM_STATE_BOOT);
    CHECK(s_a.block == ARM_BLOCK_BOOT_GRACE);
    step(true, false, true, ARM_BOOT_GRACE_MS_DEFAULT);
    CHECK(s_a.state == ARM_STATE_READY);
    CHECK(s_a.block == ARM_BLOCK_NONE);
}

static void test_switch_on_at_power_up_never_arms(void)
{
    /* The bug this module exists for: arm switch already on when the
     * receiver comes alive. */
    start(ARM_CHECKS_DEFAULT, 0U);
    for (int i = 0; i < 400; i++) {  /* 20 s of the switch sitting on */
        step(true, true, true, 50);
        CHECK(!Arming_IsArmed(&s_a));
    }
    CHECK(s_a.state == ARM_STATE_DISARMED);
    CHECK(s_a.block == ARM_BLOCK_SWITCH_ON);

    step(true, false, true, 50);  /* operator flips it off ... */
    CHECK(s_a.state == ARM_STATE_READY);
    step(true, true, true, 50);   /* ... and on again */
    CHECK(Arming_IsArmed(&s_a));
}

static void test_arms_on_edge_and_disarms_on_switch_off(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    reach_ready();
    step(true, true, true, 20);
    CHECK(s_a.state == ARM_STATE_ARMED);
    for (int i = 0; i < 50; i++) {
        step(true, true, true, 20);
        CHECK(Arming_IsArmed(&s_a));
    }
    step(true, false, true, 20);
    CHECK(s_a.state == ARM_STATE_READY);
    CHECK(!Arming_IsArmed(&s_a));
}

static void test_throttle_off_centre_refuses_until_switch_cycled(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    reach_ready();
    step(true, true, false, 20);  /* switch on, stick forward */
    CHECK(!Arming_IsArmed(&s_a));
    CHECK(s_a.block == ARM_BLOCK_THROTTLE);

    step(true, true, true, 20);   /* stick recentred, switch still on */
    CHECK(!Arming_IsArmed(&s_a)); /* must NOT arm by itself */
    CHECK(s_a.block == ARM_BLOCK_THROTTLE); /* reason stays visible */

    step(true, false, true, 20);
    step(true, true, true, 20);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_throttle_check_can_be_disabled(void)
{
    start(0U, 0U);
    reach_ready();
    step(true, true, false, 20);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_optional_imu_and_battery_checks(void)
{
    start(ARM_CHECKS_DEFAULT | ARM_CHECK_IMU | ARM_CHECK_BATTERY, 0U);
    step_sensors(true, false, true, true, 10);
    step_sensors(true, false, true, true, ARM_BOOT_GRACE_MS_DEFAULT);

    step_sensors(true, true, false, true, 20);
    CHECK(!Arming_IsArmed(&s_a));
    CHECK(s_a.block == ARM_BLOCK_IMU);
    step_sensors(true, false, true, true, 20);

    step_sensors(true, true, true, false, 20);
    CHECK(!Arming_IsArmed(&s_a));
    CHECK(s_a.block == ARM_BLOCK_BATTERY);
    step_sensors(true, false, true, true, 20);

    step_sensors(true, true, true, true, 20);
    CHECK(Arming_IsArmed(&s_a));

    /* once armed, a sensor going bad does not disarm here: that is the
     * failsafe table's decision, not the arming gate's. */
    step_sensors(true, true, false, false, 20);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_imu_battery_ignored_by_default(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    step_sensors(true, false, false, false, 10);
    step_sensors(true, false, false, false, ARM_BOOT_GRACE_MS_DEFAULT);
    step_sensors(true, true, false, false, 20);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_short_rc_dropout_keeps_armed(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    reach_ready();
    step(true, true, true, 20);
    CHECK(Arming_IsArmed(&s_a));
    for (int i = 0; i < 40; i++) {  /* 2 s without RC */
        step(false, true, true, 50);
        CHECK(Arming_IsArmed(&s_a));
    }
    step(true, true, true, 50);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_long_rc_loss_requires_rearming(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    reach_ready();
    step(true, true, true, 20);
    CHECK(Arming_IsArmed(&s_a));

    for (int i = 0; i < 120; i++) {  /* 6 s without RC */
        step(false, true, true, 50);
    }
    CHECK(s_a.state == ARM_STATE_BOOT);
    CHECK(!Arming_IsArmed(&s_a));

    /* transmitter comes back with the switch still on: must not resume */
    for (int i = 0; i < 100; i++) {
        step(true, true, true, 50);
        CHECK(!Arming_IsArmed(&s_a));
    }
    CHECK(s_a.block == ARM_BLOCK_SWITCH_ON);

    step(true, false, true, 50);
    CHECK(s_a.state == ARM_STATE_READY);
    step(true, true, true, 50);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_rc_loss_while_disarmed_forgets_the_switch(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    reach_ready();
    CHECK(s_a.state == ARM_STATE_READY);
    step(false, false, true, 50);
    CHECK(s_a.state == ARM_STATE_DISARMED);
    CHECK(s_a.block == ARM_BLOCK_NO_RC);
    step(true, true, true, 50);  /* RC returns with the switch already on */
    CHECK(!Arming_IsArmed(&s_a));
    CHECK(s_a.block == ARM_BLOCK_SWITCH_ON);
}

static void test_edge_during_boot_grace_is_ignored(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    step(true, false, true, 100);
    step(true, true, true, 100);  /* flipped on inside the grace period */
    CHECK(!Arming_IsArmed(&s_a));
    step(true, true, true, ARM_BOOT_GRACE_MS_DEFAULT);
    CHECK(!Arming_IsArmed(&s_a));  /* still on, never seen off after grace */
    CHECK(s_a.block == ARM_BLOCK_SWITCH_ON);
}

static void test_millisecond_counter_wraparound(void)
{
    /* HAL_GetTick() wraps after ~49.7 days; every interval is computed with
     * unsigned subtraction, so a boot just before the wrap must work. */
    start(ARM_CHECKS_DEFAULT, 0xFFFFFF00U);
    step(true, false, true, 10);
    step(true, false, true, ARM_BOOT_GRACE_MS_DEFAULT);  /* crosses the wrap */
    CHECK(s_a.state == ARM_STATE_READY);
    step(true, true, true, 20);
    CHECK(Arming_IsArmed(&s_a));
}

static void test_steady_inputs_are_stable(void)
{
    start(ARM_CHECKS_DEFAULT, 0U);
    reach_ready();
    for (int i = 0; i < 100; i++) {
        step(true, false, true, 20);
        CHECK(s_a.state == ARM_STATE_READY);
    }
}

int main(void)
{
    test_boot_without_rc();
    test_boot_grace_then_ready();
    test_switch_on_at_power_up_never_arms();
    test_arms_on_edge_and_disarms_on_switch_off();
    test_throttle_off_centre_refuses_until_switch_cycled();
    test_throttle_check_can_be_disabled();
    test_optional_imu_and_battery_checks();
    test_imu_battery_ignored_by_default();
    test_short_rc_dropout_keeps_armed();
    test_long_rc_loss_requires_rearming();
    test_rc_loss_while_disarmed_forgets_the_switch();
    test_edge_during_boot_grace_is_ignored();
    test_millisecond_counter_wraparound();
    test_steady_inputs_are_stable();

    if (s_failures == 0) {
        printf("arming: all checks passed\n");
        return 0;
    }
    printf("arming: %d check(s) FAILED\n", s_failures);
    return 1;
}
