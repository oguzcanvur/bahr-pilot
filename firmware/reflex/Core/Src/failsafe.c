#include "failsafe.h"
#include "esc.h"
#include "sbus.h"
#include "pi_link.h"
#include "mode_switch.h"
#include "arming.h"
#include "motor.h"
#include "battery.h"
#include "imu.h"

/* A two-position switch reads roughly CH_MIN or CH_MAX; treat anything
 * past the midpoint as "on". SBUS_GetChannel() defaults to SBUS_CH_MID
 * before any frame has been decoded, which is NOT greater than this
 * threshold — so both switches default to "off" until real data proves
 * otherwise. */
#define SWITCH_THRESHOLD SBUS_CH_MID

/* Stick inside +-THROTTLE_CENTER_BAND of trim counts as "centred" for the
 * pre-arm throttle check. */
#define THROTTLE_CENTER_BAND 0.10f

/* Dead zone on the sticks, as a fraction of full travel, so a stick that
 * does not quite spring back to centre doesn't creep the boat. */
#define RC_STICK_DEADBAND 0.05f

static uint32_t s_lastTelemetryTick;
static Arming s_arming;
static MotorState s_motor;

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

static float Throttle(const RcMapConfig *map)
{
    return NormalizeChannel(SBUS_GetChannel(map->throttle_channel), map->throttle_min,
                            map->throttle_max, map->throttle_trim, map->throttle_reversed);
}

static float Steering(const RcMapConfig *map)
{
    return NormalizeChannel(SBUS_GetChannel(map->steering_channel), map->steering_min,
                            map->steering_max, map->steering_trim, map->steering_reversed);
}

/* Immediate neutral on both ESCs and the ramp reset — never rate-limited. */
static void StopMotors(void)
{
    uint16_t m1, m2;
    Motor_Halt(&s_motor, &m1, &m2);
    ESC_SetPulse(ESC_MOTOR_1, m1);
    ESC_SetPulse(ESC_MOTOR_2, m2);
}

static void DriveMotors(uint32_t now, float left, float right)
{
    uint16_t m1, m2;
    Motor_Step(&s_motor, now, left, right, &m1, &m2);
    ESC_SetPulse(ESC_MOTOR_1, m1);
    ESC_SetPulse(ESC_MOTOR_2, m2);
}

void Failsafe_Init(void)
{
    s_lastTelemetryTick = 0U;
    Arming_Init(&s_arming, HAL_GetTick(), ARM_CHECKS_DEFAULT, ARM_BOOT_GRACE_MS_DEFAULT);
    MotorConfig cfg = MOTOR_CONFIG_DEFAULT;
    Motor_Init(&s_motor, &cfg);
    StopMotors();
}

bool Failsafe_IsArmed(void)
{
    return Arming_IsArmed(&s_arming);
}

void Failsafe_Update(void)
{
    const RcMapConfig *map = PiLink_GetRcMap();
    uint32_t now = HAL_GetTick();
    bool rcLinkUp = SBUS_IsLinkUp();

    /* Mode switch: one RC channel in six bands (ArduPilot's scheme, see
     * mode_switch.h). The Pi owns what each band means; the STM only needs
     * to know whether the band is MANUAL — then the sticks drive the motors
     * directly and nothing on the Pi can take that away. No valid RC signal
     * -> slot 0 -> never manual. */
    uint8_t modeSlot = 0U;
    if (rcLinkUp) {
        modeSlot = ModeSwitch_SlotFromUs(
            ModeSwitch_RawToUs(SBUS_GetChannel(map->mode_channel), map->mode_min, map->mode_max));
    }
    bool overrideOn = ModeSwitch_SlotIsManual(modeSlot, map->manual_slot_mask);

    /* "Armed" is no longer just "the arm switch reads on": see arming.h. */
    float throttle = Throttle(map);
    ArmInputs armInputs = {
        .now_ms = now,
        .rc_link_up = rcLinkUp,
        .switch_on = SBUS_GetChannel(map->arm_channel) > SWITCH_THRESHOLD,
        .throttle_centered = (throttle < THROTTLE_CENTER_BAND) && (throttle > -THROTTLE_CENTER_BAND),
        .imu_ok = IMU_IsValid(),
        .battery_ok = Battery_IsValid(),
    };
    Arming_Update(&s_arming, &armInputs);
    bool armed = Arming_IsArmed(&s_arming);

    if (!armed) {
        /* priority 0: not armed -> stop. Independent of the Pi/GCS "armed"
         * state — this is the one that's meant to still work if the Pi has
         * completely died */
        StopMotors();
    } else if (!rcLinkUp) {
        /* priority 2: RC lost overrides everything else, including override mode */
        StopMotors();
    } else if (overrideOn) {
        /* priority 1: manual — sticks drive the motors, no Pi involved */
        float left, right;
        Motor_Mix(Motor_ApplyDeadband(throttle, RC_STICK_DEADBAND),
                  Motor_ApplyDeadband(Steering(map), RC_STICK_DEADBAND), &left, &right);
        DriveMotors(now, left, right);
    } else if (!PiLink_IsFresh()) {
        /* priority 3/4: Pi silent (or never heard from) -> stop */
        StopMotors();
    } else {
        /* normal operation: Pi is in command. Its pulses go through the same
         * slew limit and caps as the sticks, so a runaway Pi cannot slam the
         * drive train. */
        DriveMotors(now, Motor_PulseToCommand(PiLink_GetMotor1Us()),
                    Motor_PulseToCommand(PiLink_GetMotor2Us()));
    }

    if ((now - s_lastTelemetryTick) >= RC_LINK_TELEMETRY_PERIOD_MS) {
        s_lastTelemetryTick = now;
        int16_t rollCdeg = (int16_t)(IMU_GetRollDeg() * 100.0f);
        int16_t pitchCdeg = (int16_t)(IMU_GetPitchDeg() * 100.0f);
        uint8_t armInfo = (uint8_t)(((unsigned)s_arming.state & 0x07U) |
                                    (((unsigned)s_arming.block & 0x0FU) << 4));
        PiLink_SendTelemetry(armed, rcLinkUp, overrideOn,
                              Battery_IsValid(), Battery_GetMilliVolts(),
                              IMU_IsValid(), rollCdeg, pitchCdeg, modeSlot, armInfo);
    }
}
