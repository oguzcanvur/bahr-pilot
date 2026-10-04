#include "mode_switch.h"

uint8_t ModeSwitch_SlotFromUs(uint16_t pulse_us)
{
    if (pulse_us <= 900U || pulse_us >= 2200U) {
        return 0U;
    }
    if (pulse_us <= 1230U) {
        return 1U;
    }
    if (pulse_us <= 1360U) {
        return 2U;
    }
    if (pulse_us <= 1490U) {
        return 3U;
    }
    if (pulse_us <= 1620U) {
        return 4U;
    }
    if (pulse_us <= 1749U) {
        return 5U;
    }
    return 6U;
}

uint16_t ModeSwitch_RawToUs(uint16_t raw, uint16_t min, uint16_t max)
{
    if (min >= max) {
        return 0U;
    }
    int32_t clamped = (int32_t)raw;
    if (clamped < (int32_t)min) {
        clamped = (int32_t)min;
    } else if (clamped > (int32_t)max) {
        clamped = (int32_t)max;
    }
    int32_t span = (int32_t)max - (int32_t)min;
    return (uint16_t)(1000 + ((clamped - (int32_t)min) * 1000) / span);
}

bool ModeSwitch_SlotIsManual(uint8_t slot, uint8_t manual_mask)
{
    if (slot < 1U || slot > MODE_SWITCH_SLOTS) {
        return false;
    }
    return ((manual_mask >> (slot - 1U)) & 1U) != 0U;
}
