#include "battery.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

/* !!! PLACEHOLDER — NOT CALIBRATED !!!
 * 10k:1k divider (11:1) is a common choice for a <=3.3V*11=36.3V range,
 * comfortably covering a 3-6S LiPo, but nobody has measured the real
 * resistors on this board yet (no hardware this session). Before trusting
 * any voltage this reports, measure the pack with a multimeter at a known
 * level and correct this ratio — it is a guess, not a calibrated value. */
#define BATTERY_DIVIDER_RATIO 11.0f
#define BATTERY_VREF_MV       3300.0f
#define BATTERY_ADC_MAX       4095.0f  /* 12-bit */

/* Simple exponential moving average so a single noisy ADC sample can't
 * trip a downstream low-voltage failsafe. */
#define BATTERY_FILTER_ALPHA 0.1f

static float s_filteredMv;
static bool s_valid;

void Battery_Init(void)
{
    s_filteredMv = 0.0f;
    s_valid = false;
}

void Battery_Update(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 2U) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return;
    }
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float nodeMv = ((float)raw / BATTERY_ADC_MAX) * BATTERY_VREF_MV;
    float packMv = nodeMv * BATTERY_DIVIDER_RATIO;

    if (!s_valid) {
        s_filteredMv = packMv;  /* snap to first real reading, don't ramp from 0 */
        s_valid = true;
    } else {
        s_filteredMv += BATTERY_FILTER_ALPHA * (packMv - s_filteredMv);
    }
}

uint16_t Battery_GetMilliVolts(void)
{
    if (!s_valid) {
        return 0U;
    }
    if (s_filteredMv < 0.0f) {
        return 0U;
    }
    if (s_filteredMv > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)s_filteredMv;
}

bool Battery_IsValid(void)
{
    return s_valid;
}
