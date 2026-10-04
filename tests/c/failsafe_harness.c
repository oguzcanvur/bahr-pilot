/* Drives the REAL firmware/reflex/Core/Src/failsafe.c on a host PC, together
 * with the real arming.c, motor.c and mode_switch.c. Only the hardware-facing
 * pieces (SBUS receiver, ESC outputs, Pi link, battery, IMU) are stubbed
 * below. tests/test_c_firmware_logic.py runs scenarios against it, so the
 * failsafe priority table is exercised instead of merely compiled.
 *
 * One command per stdin line; each gets exactly one reply line:
 *   tick <ms>                 advance the fake clock
 *   ch <idx> <value>          set an SBUS channel value
 *   link <0|1>                SBUS link up / down
 *   pi <fresh 0|1> <m1> <m2>  Pi link freshness and its last motor pulses
 *   sensors <imu 0|1> <bat 0|1>
 *   mask <manual_slot_mask>   which mode-switch bands are MANUAL
 *   update                    run Failsafe_Update(); replies
 *                             "out <esc1> <esc2> <armed> <state> <block>"
 *   tele                      replies with the arguments of the last
 *                             PiLink_SendTelemetry() call
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "failsafe.h"
#include "esc.h"
#include "sbus.h"
#include "pi_link.h"
#include "battery.h"
#include "imu.h"

static uint32_t s_tick;
static uint16_t s_channels[SBUS_NUM_CHANNELS];
static bool s_link;
static bool s_piFresh;
static uint16_t s_piM1 = 1500, s_piM2 = 1500;
static bool s_imuOk, s_batOk;
static uint16_t s_esc[2] = {1500, 1500};
static RcMapConfig s_map = {
    .throttle_channel = 2U, .steering_channel = 0U, .arm_channel = 5U, .mode_channel = 4U,
    .throttle_min = SBUS_CH_MIN, .throttle_max = SBUS_CH_MAX, .throttle_trim = SBUS_CH_MID,
    .steering_min = SBUS_CH_MIN, .steering_max = SBUS_CH_MAX, .steering_trim = SBUS_CH_MID,
    .throttle_reversed = false, .steering_reversed = false,
    .mode_min = SBUS_CH_MIN, .mode_max = SBUS_CH_MAX, .manual_slot_mask = 0x07U,
};
static struct {
    bool armed, rc, override, bat, imu;
    unsigned mv;
    int roll, pitch, slot, arm_info;
    int calls;
} s_tele;

uint32_t HAL_GetTick(void) { return s_tick; }

uint16_t SBUS_GetChannel(uint8_t idx) { return idx < SBUS_NUM_CHANNELS ? s_channels[idx] : SBUS_CH_MID; }
bool SBUS_IsLinkUp(void) { return s_link; }

void ESC_SetPulse(ESC_Motor motor, uint16_t pulse_us) { s_esc[motor == ESC_MOTOR_1 ? 0 : 1] = pulse_us; }

const RcMapConfig *PiLink_GetRcMap(void) { return &s_map; }
bool PiLink_IsFresh(void) { return s_piFresh; }
uint16_t PiLink_GetMotor1Us(void) { return s_piM1; }
uint16_t PiLink_GetMotor2Us(void) { return s_piM2; }

void PiLink_SendTelemetry(bool armed, bool rcLinkUp, bool overrideActive, bool batteryValid,
                           uint16_t batteryMv, bool imuValid, int16_t rollCdeg, int16_t pitchCdeg,
                           uint8_t modeSlot, uint8_t armInfo)
{
    s_tele.armed = armed;
    s_tele.rc = rcLinkUp;
    s_tele.override = overrideActive;
    s_tele.bat = batteryValid;
    s_tele.mv = batteryMv;
    s_tele.imu = imuValid;
    s_tele.roll = rollCdeg;
    s_tele.pitch = pitchCdeg;
    s_tele.slot = modeSlot;
    s_tele.arm_info = armInfo;
    s_tele.calls++;
}

bool Battery_IsValid(void) { return s_batOk; }
uint16_t Battery_GetMilliVolts(void) { return s_batOk ? 12000U : 0U; }
bool IMU_IsValid(void) { return s_imuOk; }
float IMU_GetRollDeg(void) { return 0.0f; }
float IMU_GetPitchDeg(void) { return 0.0f; }

int main(void)
{
    for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
        s_channels[i] = SBUS_CH_MID;
    }
    Failsafe_Init();

    char line[256];
    while (fgets(line, sizeof line, stdin) != NULL) {
        char cmd[16] = {0};
        sscanf(line, "%15s", cmd);

        if (strcmp(cmd, "tick") == 0) {
            unsigned ms = 0;
            sscanf(line, "%*s %u", &ms);
            s_tick += ms;
            printf("ok\n");
        } else if (strcmp(cmd, "ch") == 0) {
            unsigned idx = 0, value = 0;
            sscanf(line, "%*s %u %u", &idx, &value);
            if (idx < SBUS_NUM_CHANNELS) {
                s_channels[idx] = (uint16_t)value;
            }
            printf("ok\n");
        } else if (strcmp(cmd, "link") == 0) {
            int v = 0;
            sscanf(line, "%*s %d", &v);
            s_link = v != 0;
            printf("ok\n");
        } else if (strcmp(cmd, "pi") == 0) {
            int fresh = 0;
            unsigned m1 = 1500, m2 = 1500;
            sscanf(line, "%*s %d %u %u", &fresh, &m1, &m2);
            s_piFresh = fresh != 0;
            s_piM1 = (uint16_t)m1;
            s_piM2 = (uint16_t)m2;
            printf("ok\n");
        } else if (strcmp(cmd, "sensors") == 0) {
            int imu = 0, bat = 0;
            sscanf(line, "%*s %d %d", &imu, &bat);
            s_imuOk = imu != 0;
            s_batOk = bat != 0;
            printf("ok\n");
        } else if (strcmp(cmd, "mask") == 0) {
            unsigned m = 0;
            sscanf(line, "%*s %u", &m);
            s_map.manual_slot_mask = (uint8_t)m;
            printf("ok\n");
        } else if (strcmp(cmd, "update") == 0) {
            Failsafe_Update();
            printf("out %u %u %d\n", s_esc[0], s_esc[1], Failsafe_IsArmed());
        } else if (strcmp(cmd, "tele") == 0) {
            printf("tele %d %d %d %d %u %d %d %d %d %d %d\n", s_tele.armed, s_tele.rc, s_tele.override,
                   s_tele.bat, s_tele.mv, s_tele.imu, s_tele.roll, s_tele.pitch, s_tele.slot,
                   s_tele.arm_info, s_tele.calls);
        } else {
            printf("unknown\n");
        }
        fflush(stdout);
    }
    return 0;
}
