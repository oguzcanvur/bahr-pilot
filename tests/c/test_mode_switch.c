/* Host-side test for firmware/reflex/Core/Src/mode_switch.c (pure logic, no
 * HAL). Kept outside the CubeIDE project folder so the IDE never tries to
 * link a second main().
 *
 * Build and run from the repository root with any host C compiler:
 *   gcc -Wall -Wextra -Ifirmware/reflex/Core/Inc \
 *       tests/c/test_mode_switch.c firmware/reflex/Core/Src/mode_switch.c \
 *       -o test_mode_switch && ./test_mode_switch
 * tests/test_c_firmware_logic.py does exactly that when a compiler is found.
 */
#include <stdio.h>
#include <stdlib.h>

#include "mode_switch.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            s_failures++;                                                    \
        }                                                                    \
    } while (0)

static void test_band_edges(void)
{
    /* Both sides of every ArduPilot band edge. */
    CHECK(ModeSwitch_SlotFromUs(901U) == 1U);
    CHECK(ModeSwitch_SlotFromUs(1000U) == 1U);
    CHECK(ModeSwitch_SlotFromUs(1230U) == 1U);
    CHECK(ModeSwitch_SlotFromUs(1231U) == 2U);
    CHECK(ModeSwitch_SlotFromUs(1360U) == 2U);
    CHECK(ModeSwitch_SlotFromUs(1361U) == 3U);
    CHECK(ModeSwitch_SlotFromUs(1490U) == 3U);
    CHECK(ModeSwitch_SlotFromUs(1491U) == 4U);
    CHECK(ModeSwitch_SlotFromUs(1500U) == 4U);
    CHECK(ModeSwitch_SlotFromUs(1620U) == 4U);
    CHECK(ModeSwitch_SlotFromUs(1621U) == 5U);
    CHECK(ModeSwitch_SlotFromUs(1749U) == 5U);
    CHECK(ModeSwitch_SlotFromUs(1750U) == 6U);
    CHECK(ModeSwitch_SlotFromUs(2000U) == 6U);
    CHECK(ModeSwitch_SlotFromUs(2199U) == 6U);
}

static void test_invalid_pulse(void)
{
    CHECK(ModeSwitch_SlotFromUs(0U) == 0U);
    CHECK(ModeSwitch_SlotFromUs(900U) == 0U);
    CHECK(ModeSwitch_SlotFromUs(2200U) == 0U);
    CHECK(ModeSwitch_SlotFromUs(65535U) == 0U);
}

static void test_default_sbus_calibration(void)
{
    /* Default RCn_MIN/MAX are the full SBUS range 172..1811. A three
     * position switch sits at roughly min / mid / max and must land in
     * slots 1 / 4 / 6 (so MODE1, MODE4, MODE6 are the three used ones). */
    CHECK(ModeSwitch_RawToUs(172U, 172U, 1811U) == 1000U);
    CHECK(ModeSwitch_RawToUs(1811U, 172U, 1811U) == 2000U);
    uint16_t mid = ModeSwitch_RawToUs(992U, 172U, 1811U);
    CHECK(mid >= 1499U && mid <= 1501U);

    CHECK(ModeSwitch_SlotFromUs(ModeSwitch_RawToUs(172U, 172U, 1811U)) == 1U);
    CHECK(ModeSwitch_SlotFromUs(ModeSwitch_RawToUs(992U, 172U, 1811U)) == 4U);
    CHECK(ModeSwitch_SlotFromUs(ModeSwitch_RawToUs(1811U, 172U, 1811U)) == 6U);
}

static void test_raw_clamped_to_calibration(void)
{
    /* A stick travelling a little past the calibrated ends must clamp,
     * not wrap around into the opposite band. */
    CHECK(ModeSwitch_RawToUs(0U, 172U, 1811U) == 1000U);
    CHECK(ModeSwitch_RawToUs(2047U, 172U, 1811U) == 2000U);
}

static void test_bad_calibration_is_invalid(void)
{
    CHECK(ModeSwitch_RawToUs(992U, 1811U, 172U) == 0U);
    CHECK(ModeSwitch_RawToUs(992U, 500U, 500U) == 0U);
    CHECK(ModeSwitch_SlotFromUs(ModeSwitch_RawToUs(992U, 500U, 500U)) == 0U);
}

static void test_manual_mask(void)
{
    /* Default: slots 1-3 manual (MODE1..3 = MANUAL), 4-6 not. */
    uint8_t mask = 0x07U;
    for (uint8_t slot = 1U; slot <= 3U; slot++) {
        CHECK(ModeSwitch_SlotIsManual(slot, mask));
    }
    for (uint8_t slot = 4U; slot <= 6U; slot++) {
        CHECK(!ModeSwitch_SlotIsManual(slot, mask));
    }
    /* A missing signal must never be treated as manual, even with every
     * bit set. */
    CHECK(!ModeSwitch_SlotIsManual(0U, 0xFFU));
    CHECK(!ModeSwitch_SlotIsManual(7U, 0xFFU));
    CHECK(!ModeSwitch_SlotIsManual(1U, 0x00U));
}

int main(void)
{
    test_band_edges();
    test_invalid_pulse();
    test_default_sbus_calibration();
    test_raw_clamped_to_calibration();
    test_bad_calibration_is_invalid();
    test_manual_mask();

    if (s_failures == 0) {
        printf("mode_switch: all checks passed\n");
        return 0;
    }
    printf("mode_switch: %d check(s) FAILED\n", s_failures);
    return 1;
}
