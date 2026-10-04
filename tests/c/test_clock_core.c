/* Host-side test for firmware/reflex/Core/Src/clock_core.c (pure logic).
 * Built and run by tests/test_c_firmware_logic.py. */
#include <stdio.h>

#include "clock.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            s_failures++;                                                    \
        }                                                                    \
    } while (0)

static void test_counts_from_first_reading(void)
{
    ClockCore c = {0};
    ClockCore_Update(&c, 1000000U);   /* the counter may start anywhere */
    CHECK(ClockCore_Us(&c, 170U) == 0U);
    ClockCore_Update(&c, 1000000U + 170U * 1000U);
    CHECK(ClockCore_Us(&c, 170U) == 1000U);
}

static void test_survives_32_bit_wraparound(void)
{
    ClockCore c = {0};
    ClockCore_Update(&c, 0xFFFFFF00U);
    ClockCore_Update(&c, 0x00000100U);   /* wrapped: 0x200 cycles passed */
    CHECK(c.total_cycles == 0x200U);
}

static void test_many_wraps_stay_monotonic(void)
{
    /* 170 MHz: the counter wraps every ~25.26 s. Simulate 10 minutes read
     * every 5 ms and check time never goes backwards and ends up right. */
    ClockCore c = {0};
    uint32_t counter = 12345U;
    ClockCore_Update(&c, counter);
    uint64_t previous = 0U;
    uint64_t step_cycles = 170U * 5000U;           /* 5 ms */
    for (uint64_t i = 0U; i < 120000U; i++) {      /* 600 s */
        counter += (uint32_t)step_cycles;
        ClockCore_Update(&c, counter);
        uint64_t now = ClockCore_Us(&c, 170U);
        CHECK(now >= previous);
        previous = now;
    }
    CHECK(previous == 600U * 1000U * 1000U);
}

static void test_zero_cycles_per_us_is_safe(void)
{
    ClockCore c = {0};
    ClockCore_Update(&c, 5U);
    ClockCore_Update(&c, 500U);
    CHECK(ClockCore_Us(&c, 0U) == 0U);
}

int main(void)
{
    test_counts_from_first_reading();
    test_survives_32_bit_wraparound();
    test_many_wraps_stay_monotonic();
    test_zero_cycles_per_us_is_safe();

    if (s_failures == 0) {
        printf("clock_core: all checks passed\n");
        return 0;
    }
    printf("clock_core: %d check(s) FAILED\n", s_failures);
    return 1;
}
