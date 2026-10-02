#include "failsafe.h"
#include "esc.h"
#include "sbus.h"
#include "pi_link.h"
#include "battery.h"
#include "imu.h"

/* A two-position switch reads roughly CH_MIN or CH_MAX; treat anything
 * past the midpoint as "on". SBUS_GetChannel() defaults to SBUS_CH_MID
 * before any frame has been decoded, which is NOT greater than this
 * threshold — so both switches default to "off" until real data proves
 * otherwise. */
#define SWITCH_THRESHOLD SBUS_CH_MID

#define MIX_SPEED_SPAN_US 500.0f

static uint32_t s_lastTelemetryTick;
static bool s_armed;

static float NormalizeChannel(uint16_t raw, uint16_t min, uint16_t max, uint16_t trim, bool reversed)
{
    int32_t diff = (int32_t)raw - (int32_t)trim;
    float norm;
    if (diff >= 0) {
        int32_t span = (int32_t)max - (int32_t)trim;
        norm = (span > 0) ? ((float)diff / (float)span) : 0.0f;
    } else {
        int32_t span = (int32_t)trim - (int32_t)min;
        norm = (span > 0) ? ((float)diff / (float)span) : 0.0f;
    }
    if (norm > 1.0f) {
        norm = 1.0f;
    } else if (norm < -1.0f) {
        norm = -1.0f;
    }
    return reversed ? -norm : norm;
}

static void DriveFromRc(const RcMapConfig *map)
{
    float throttle = NormalizeChannel(
        SBUS_GetChannel(map->throttle_channel), map->throttle_min, map->throttle_max,
        map->throttle_trim, map->throttle_reversed);
    float steering = NormalizeChannel(
        SBUS_GetChannel(map->steering_channel), map->steering_min, map->steering_max,
        map->steering_trim, map->steering_reversed);

    float m1 = (float)ESC_PULSE_NEUTRAL_US + MIX_SPEED_SPAN_US * (throttle - steering);
    float m2 = (float)ESC_PULSE_NEUTRAL_US + MIX_SPEED_SPAN_US * (throttle + steering);
    ESC_SetPulse(ESC_MOTOR_1, (uint16_t)m1);
    ESC_SetPulse(ESC_MOTOR_2, (uint16_t)m2);
}

void Failsafe_Init(void)
{
    s_lastTelemetryTick = 0U;
    s_armed = false;
    ESC_Stop();
}

bool Failsafe_IsArmed(void)
{
    return s_armed;
}

void Failsafe_Update(void)
{
    const RcMapConfig *map = PiLink_GetRcMap();

    bool armed = SBUS_GetChannel(map->arm_channel) > SWITCH_THRESHOLD;
    s_armed = armed;
    bool rcLinkUp = SBUS_IsLinkUp();
    bool overrideOn = rcLinkUp && SBUS_GetChannel(map->override_channel) > SWITCH_THRESHOLD;

    if (!armed) {
        /* priority 0: physical arm switch off overrides everything below,
         * independent of the Pi/GCS "armed" state — this is the one
         * that's meant to still work if the Pi has completely died */
        ESC_Stop();
    } else if (!rcLinkUp) {
        /* priority 2: RC lost overrides everything else, including override mode */
        ESC_Stop();
    } else if (overrideOn) {
        /* priority 1: manual override drives the motors via throttle+steering mix */
        DriveFromRc(map);
    } else if (!PiLink_IsFresh()) {
        /* priority 3/4: Pi silent (or never heard from) -> stop */
        ESC_Stop();
    } else {
        /* normal operation: Pi is in command */
        ESC_SetPulse(ESC_MOTOR_1, PiLink_GetMotor1Us());
        ESC_SetPulse(ESC_MOTOR_2, PiLink_GetMotor2Us());
    }

    uint32_t now = HAL_GetTick();
    if ((now - s_lastTelemetryTick) >= RC_LINK_TELEMETRY_PERIOD_MS) {
        s_lastTelemetryTick = now;
        int16_t rollCdeg = (int16_t)(IMU_GetRollDeg() * 100.0f);
        int16_t pitchCdeg = (int16_t)(IMU_GetPitchDeg() * 100.0f);
        PiLink_SendTelemetry(armed, rcLinkUp, overrideOn,
                              Battery_IsValid(), Battery_GetMilliVolts(),
                              IMU_IsValid(), rollCdeg, pitchCdeg);
    }
}
