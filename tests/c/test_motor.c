/* Host-side test for firmware/reflex/Core/Src/motor.c (pure logic).
 * Built and run by tests/test_c_firmware_logic.py. */
#include <math.h>
#include <stdio.h>

#include "motor.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            s_failures++;                                                    \
        }                                                                    \
    } while (0)

#define NEAR(a, b) (fabsf((float)(a) - (float)(b)) < 1e-4f)

static void test_deadband(void)
{
    CHECK(NEAR(Motor_ApplyDeadband(0.0f, 0.1f), 0.0f));
    CHECK(NEAR(Motor_ApplyDeadband(0.09f, 0.1f), 0.0f));
    CHECK(NEAR(Motor_ApplyDeadband(-0.1f, 0.1f), 0.0f));
    /* continuous at the edge and full scale at full deflection */
    CHECK(Motor_ApplyDeadband(0.1001f, 0.1f) < 0.001f);
    CHECK(NEAR(Motor_ApplyDeadband(1.0f, 0.1f), 1.0f));
    CHECK(NEAR(Motor_ApplyDeadband(-1.0f, 0.1f), -1.0f));
    CHECK(NEAR(Motor_ApplyDeadband(0.55f, 0.1f), 0.5f));
    /* garbage in is clamped, not amplified */
    CHECK(NEAR(Motor_ApplyDeadband(5.0f, 0.1f), 1.0f));
    CHECK(NEAR(Motor_ApplyDeadband(0.5f, 0.0f), 0.5f));
    CHECK(NEAR(Motor_ApplyDeadband(0.5f, 1.0f), 0.0f));
}

static void test_mix_directions(void)
{
    float l, r;
    Motor_Mix(0.5f, 0.0f, &l, &r);          /* straight ahead */
    CHECK(NEAR(l, 0.5f) && NEAR(r, 0.5f));

    Motor_Mix(0.5f, 0.2f, &l, &r);          /* steering right: left faster */
    CHECK(l > r);
    CHECK(NEAR(l, 0.7f) && NEAR(r, 0.3f));

    Motor_Mix(0.5f, -0.2f, &l, &r);         /* steering left: right faster */
    CHECK(r > l);

    Motor_Mix(0.0f, 0.6f, &l, &r);          /* pivot right on the spot */
    CHECK(NEAR(l, 0.6f) && NEAR(r, -0.6f));

    Motor_Mix(-0.5f, 0.0f, &l, &r);         /* reverse */
    CHECK(NEAR(l, -0.5f) && NEAR(r, -0.5f));
}

static void test_mix_desaturation_keeps_the_turn_ratio(void)
{
    float l, r;
    Motor_Mix(1.0f, 0.5f, &l, &r);          /* raw: 1.5 / 0.5 */
    CHECK(fabsf(l) <= 1.0f && fabsf(r) <= 1.0f);
    CHECK(NEAR(l / r, 3.0f));               /* ratio of 1.5 : 0.5 preserved */
    CHECK(l > r);

    Motor_Mix(1.0f, 1.0f, &l, &r);          /* raw: 2.0 / 0.0 */
    CHECK(NEAR(l, 1.0f) && NEAR(r, 0.0f));

    Motor_Mix(-1.0f, -1.0f, &l, &r);        /* raw: -2.0 / 0.0 */
    CHECK(NEAR(l, -1.0f) && NEAR(r, 0.0f));
}

static void test_zero_command_is_exact_neutral(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    for (uint32_t t = 0; t < 2000; t += 10) {
        Motor_Step(&m, t, 0.0f, 0.0f, &a, &b);
        CHECK(a == 1500 && b == 1500);
    }
}

static void test_slew_limits_rate_of_change(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;   /* 100 %/s */
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    Motor_Step(&m, 0, 1.0f, 1.0f, &a, &b);
    CHECK(a == 1500);                          /* first step: no time elapsed */
    Motor_Step(&m, 100, 1.0f, 1.0f, &a, &b);
    CHECK(a == 1550 && b == 1550);             /* 0.1 s at 100 %/s -> 10 % */
    Motor_Step(&m, 500, 1.0f, 1.0f, &a, &b);   /* caps dt at 250 ms: +25 % */
    CHECK(a == 1675);
    for (uint32_t t = 600; t <= 2000; t += 100) {
        Motor_Step(&m, t, 1.0f, 1.0f, &a, &b);
    }
    CHECK(a == 2000 && b == 2000);
}

static void test_full_reversal_ramps_through_neutral(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    uint32_t t = 0;
    Motor_Step(&m, t, 1.0f, 1.0f, &a, &b);
    for (; t <= 2000; t += 20) {               /* reach full ahead */
        Motor_Step(&m, t, 1.0f, 1.0f, &a, &b);
    }
    t -= 20;                                    /* t is now the last stepped time */
    CHECK(a == 2000);

    int crossed_neutral = 0;
    uint16_t previous = a;
    for (int i = 0; i < 300; i++) {            /* now demand full astern */
        t += 20;
        Motor_Step(&m, t, -1.0f, -1.0f, &a, &b);
        int step = (int)a - (int)previous;
        CHECK(step >= -11 && step <= 0);       /* at most ~100 %/s * 20 ms = 10 us */
        if (a == 1500) {
            crossed_neutral = 1;
        }
        previous = a;
    }
    CHECK(crossed_neutral);
    CHECK(a == 1000);
}

static void test_unlimited_slew_when_disabled(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    cfg.slew_pct_per_s = 0U;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    Motor_Step(&m, 0, 1.0f, -1.0f, &a, &b);
    CHECK(a == 2000 && b == 1000);
}

static void test_halt_is_immediate_and_resets_the_ramp(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    for (uint32_t t = 0; t < 3000; t += 20) {
        Motor_Step(&m, t, 1.0f, 1.0f, &a, &b);
    }
    CHECK(a == 2000);

    Motor_Halt(&m, &a, &b);                    /* not rate-limited */
    CHECK(a == 1500 && b == 1500);

    Motor_Step(&m, 5000, 1.0f, 1.0f, &a, &b);  /* resuming starts from zero */
    CHECK(a == 1500);
    Motor_Step(&m, 5020, 1.0f, 1.0f, &a, &b);
    CHECK(a == 1510);
}

static void test_max_cap(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    cfg.max_pct = 60U;
    cfg.slew_pct_per_s = 0U;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    Motor_Step(&m, 0, 1.0f, -1.0f, &a, &b);
    CHECK(a == 1800 && b == 1200);
}

static void test_reversed_motor(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    cfg.slew_pct_per_s = 0U;
    cfg.reversed[1] = true;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    Motor_Step(&m, 0, 0.5f, 0.5f, &a, &b);
    CHECK(a == 1750 && b == 1250);
}

static void test_output_always_inside_esc_range(void)
{
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    cfg.slew_pct_per_s = 0U;
    MotorState m;
    Motor_Init(&m, &cfg);
    uint16_t a, b;
    const float wild[] = {-1e9f, -3.0f, -1.0f, -0.0001f, 0.0f, 0.0001f, 1.0f, 3.0f, 1e9f};
    for (unsigned i = 0; i < sizeof wild / sizeof wild[0]; i++) {
        for (unsigned j = 0; j < sizeof wild / sizeof wild[0]; j++) {
            Motor_Step(&m, (uint32_t)(i * 10 + j), wild[i], wild[j], &a, &b);
            CHECK(a >= 1000 && a <= 2000 && b >= 1000 && b <= 2000);
        }
    }
}

static void test_pulse_to_command(void)
{
    CHECK(NEAR(Motor_PulseToCommand(1500), 0.0f));
    CHECK(NEAR(Motor_PulseToCommand(2000), 1.0f));
    CHECK(NEAR(Motor_PulseToCommand(1000), -1.0f));
    CHECK(NEAR(Motor_PulseToCommand(1750), 0.5f));
    CHECK(NEAR(Motor_PulseToCommand(0), -1.0f));      /* garbage clamps */
    CHECK(NEAR(Motor_PulseToCommand(65535), 1.0f));
}

int main(void)
{
    test_deadband();
    test_mix_directions();
    test_mix_desaturation_keeps_the_turn_ratio();
    test_zero_command_is_exact_neutral();
    test_slew_limits_rate_of_change();
    test_full_reversal_ramps_through_neutral();
    test_unlimited_slew_when_disabled();
    test_halt_is_immediate_and_resets_the_ramp();
    test_max_cap();
    test_reversed_motor();
    test_output_always_inside_esc_range();
    test_pulse_to_command();

    if (s_failures == 0) {
        printf("motor: all checks passed\n");
        return 0;
    }
    printf("motor: %d check(s) FAILED\n", s_failures);
    return 1;
}
