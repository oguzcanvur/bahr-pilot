/* Drives the REAL firmware/reflex/Core/Src/pi_link.c on a host PC, with the
 * HAL replaced by the stubs below. tests/test_pi_link_c.py feeds it frames
 * built by the real Python NucleoLink and decodes what it transmits, so a
 * byte-layout disagreement between the two sides fails a test instead of
 * silently desyncing on the boat.
 *
 * One command per stdin line; replies are one line each:
 *   rx <hex>                 feed received bytes through the USART3 RX ISR
 *   tick <ms>                advance the fake millisecond clock
 *   process <armed 0|1>      PiLink_Process(); replies "saves <n>"
 *   chan <idx> <value>       set the fake SBUS channel value
 *   map                      replies with the active RC map
 *   motors                   replies "motors <fresh> <m1_us> <m2_us>"
 *   tx <armed> <rc> <override> <bat_valid> <mv> <imu_valid> <roll> <pitch> <slot> <arm_info>
 *                            send a telemetry frame, run the TX interrupt chain
 *                            to completion; replies "tx <hex of everything sent>"
 *   imu <t_us> <qx qy qz qw> <gx gy gz> <ax ay az> <flags> <qacc>
 *                            same for an IMU frame; replies "imu <hex>"
 *   burst <n>                queue n telemetry+IMU frame pairs WITHOUT letting
 *                            the UART drain in between (ring-buffer pressure),
 *                            then drain; replies "burst <hex>"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pi_link.h"

UART_HandleTypeDef huart3;

static uint8_t *s_rxPtr;
static uint32_t s_tick;
static int s_saveCount;
static RcMapConfig s_saved;
static bool s_haveSaved;
static uint8_t s_stream[8192];
static unsigned s_streamLen;
static int s_txPending;
static uint16_t s_channels[SBUS_NUM_CHANNELS];

uint32_t HAL_GetTick(void) { return s_tick; }

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart;
    (void)Size;
    s_rxPtr = pData;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart;
    if (s_streamLen + Size <= sizeof s_stream) {
        memcpy(&s_stream[s_streamLen], pData, Size);
        s_streamLen += Size;
    }
    s_txPending = 1;
    return HAL_OK;
}

uint16_t SBUS_GetChannel(uint8_t idx) { return idx < SBUS_NUM_CHANNELS ? s_channels[idx] : SBUS_CH_MID; }

bool Settings_LoadRcMap(RcMapConfig *out)
{
    if (!s_haveSaved) {
        return false;
    }
    *out = s_saved;
    return true;
}

void Settings_SaveRcMap(const RcMapConfig *map)
{
    s_saved = *map;
    s_haveSaved = true;
    s_saveCount++;
}

/* Plays the UART's TX-complete interrupt until the queue is empty, then
 * prints and clears everything that went out on the wire. */
static void drain_and_print(const char *label)
{
    while (s_txPending) {
        s_txPending = 0;
        PiLink_UART_TxCpltCallback(&huart3);
    }
    printf("%s ", label);
    for (unsigned i = 0; i < s_streamLen; i++) {
        printf("%02x", s_stream[i]);
    }
    printf("\n");
    s_streamLen = 0;
}

static int hex_to_bytes(const char *hex, uint8_t *out, int max)
{
    int n = 0;
    while (hex[0] != '\0' && hex[1] != '\0' && n < max) {
        char pair[3] = {hex[0], hex[1], '\0'};
        out[n++] = (uint8_t)strtoul(pair, NULL, 16);
        hex += 2;
    }
    return n;
}

int main(void)
{
    huart3.Instance = USART3;
    for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
        s_channels[i] = SBUS_CH_MID;
    }
    PiLink_Init();

    char line[512];
    while (fgets(line, sizeof line, stdin) != NULL) {
        char cmd[16] = {0};
        char arg[400] = {0};
        sscanf(line, "%15s %399s", cmd, arg);

        if (strcmp(cmd, "rx") == 0) {
            uint8_t bytes[128];
            int n = hex_to_bytes(arg, bytes, (int)sizeof bytes);
            for (int i = 0; i < n; i++) {
                *s_rxPtr = bytes[i];
                PiLink_UART_RxCpltCallback(&huart3);
            }
            printf("ok\n");
        } else if (strcmp(cmd, "tick") == 0) {
            s_tick += (uint32_t)strtoul(arg, NULL, 10);
            printf("ok\n");
        } else if (strcmp(cmd, "process") == 0) {
            PiLink_Process(atoi(arg) != 0);
            printf("saves %d\n", s_saveCount);
        } else if (strcmp(cmd, "chan") == 0) {
            unsigned idx = 0, value = 0;
            sscanf(line, "%*s %u %u", &idx, &value);
            if (idx < SBUS_NUM_CHANNELS) {
                s_channels[idx] = (uint16_t)value;
            }
            printf("ok\n");
        } else if (strcmp(cmd, "map") == 0) {
            const RcMapConfig *m = PiLink_GetRcMap();
            printf("map %u %u %u %u %u %u %u %u %u %u %d %d %u %u %u\n",
                   m->throttle_channel, m->steering_channel, m->arm_channel, m->mode_channel,
                   m->throttle_min, m->throttle_max, m->throttle_trim,
                   m->steering_min, m->steering_max, m->steering_trim,
                   m->throttle_reversed, m->steering_reversed,
                   m->mode_min, m->mode_max, m->manual_slot_mask);
        } else if (strcmp(cmd, "motors") == 0) {
            printf("motors %d %u %u\n", PiLink_IsFresh(), PiLink_GetMotor1Us(), PiLink_GetMotor2Us());
        } else if (strcmp(cmd, "tx") == 0) {
            int armed, rc, ovr, bat, imu, roll, pitch, slot, arm_info = 0;
            unsigned mv;
            sscanf(line, "%*s %d %d %d %d %u %d %d %d %d %d", &armed, &rc, &ovr, &bat, &mv, &imu,
                   &roll, &pitch, &slot, &arm_info);
            PiLink_SendTelemetry(armed != 0, rc != 0, ovr != 0, bat != 0, (uint16_t)mv, imu != 0,
                                 (int16_t)roll, (int16_t)pitch, (uint8_t)slot, (uint8_t)arm_info);
            drain_and_print("tx");
        } else if (strcmp(cmd, "imu") == 0) {
            ImuSample s;
            memset(&s, 0, sizeof s);
            unsigned t_us = 0, flags = 0, qacc = 0;
            sscanf(line, "%*s %u %f %f %f %f %f %f %f %f %f %f %u %u", &t_us, &s.quat[0], &s.quat[1],
                   &s.quat[2], &s.quat[3], &s.gyro[0], &s.gyro[1], &s.gyro[2], &s.accel[0], &s.accel[1],
                   &s.accel[2], &flags, &qacc);
            s.t_us = t_us;
            s.quat_valid = (flags & 1U) != 0;
            s.gyro_valid = (flags & 2U) != 0;
            s.accel_valid = (flags & 4U) != 0;
            s.quat_accuracy = (uint8_t)qacc;
            PiLink_SendImu(&s);
            drain_and_print("imu");
        } else if (strcmp(cmd, "burst") == 0) {
            int n = atoi(arg);
            ImuSample s;
            memset(&s, 0, sizeof s);
            s.quat[3] = 1.0f;
            s.quat_valid = true;
            for (int i = 0; i < n; i++) {
                s.t_us = (uint32_t)i;
                PiLink_SendTelemetry(true, true, false, true, 12000, true, 0, 0, 4, 0);
                PiLink_SendImu(&s);
            }
            drain_and_print("burst");
        } else {
            printf("unknown\n");
        }
        fflush(stdout);
    }
    return 0;
}
