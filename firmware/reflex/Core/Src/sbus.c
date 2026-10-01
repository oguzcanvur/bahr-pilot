#include "sbus.h"
#include <string.h>

#define SBUS_FRAME_LEN   25U
#define SBUS_START_BYTE  0x0FU
#define SBUS_END_BYTE    0x00U
#define SBUS_FLAG_FRAME_LOST (1U << 2)
#define SBUS_FLAG_FAILSAFE   (1U << 3)

extern UART_HandleTypeDef huart1;

static uint8_t s_rxFrame[SBUS_FRAME_LEN];
static uint8_t s_rxIndex;
static uint8_t s_rxByte;

static uint16_t s_channels[SBUS_NUM_CHANNELS];
static volatile bool s_hasFrame;
static volatile bool s_failsafe;
static volatile uint32_t s_lastFrameTick;

static void DecodeFrame(const uint8_t *f)
{
    const uint8_t *d = &f[1]; /* 22 packed payload bytes */

    s_channels[0]  = (uint16_t)((d[0]       | d[1] << 8))                  & 0x07FFU;
    s_channels[1]  = (uint16_t)((d[1] >> 3  | d[2] << 5))                  & 0x07FFU;
    s_channels[2]  = (uint16_t)((d[2] >> 6  | d[3] << 2  | d[4] << 10))    & 0x07FFU;
    s_channels[3]  = (uint16_t)((d[4] >> 1  | d[5] << 7))                  & 0x07FFU;
    s_channels[4]  = (uint16_t)((d[5] >> 4  | d[6] << 4))                  & 0x07FFU;
    s_channels[5]  = (uint16_t)((d[6] >> 7  | d[7] << 1  | d[8] << 9))     & 0x07FFU;
    s_channels[6]  = (uint16_t)((d[8] >> 2  | d[9] << 6))                  & 0x07FFU;
    s_channels[7]  = (uint16_t)((d[9] >> 5  | d[10] << 3))                 & 0x07FFU;
    s_channels[8]  = (uint16_t)((d[11]      | d[12] << 8))                 & 0x07FFU;
    s_channels[9]  = (uint16_t)((d[12] >> 3 | d[13] << 5))                 & 0x07FFU;
    s_channels[10] = (uint16_t)((d[13] >> 6 | d[14] << 2 | d[15] << 10))   & 0x07FFU;
    s_channels[11] = (uint16_t)((d[15] >> 1 | d[16] << 7))                 & 0x07FFU;
    s_channels[12] = (uint16_t)((d[16] >> 4 | d[17] << 4))                 & 0x07FFU;
    s_channels[13] = (uint16_t)((d[17] >> 7 | d[18] << 1 | d[19] << 9))    & 0x07FFU;
    s_channels[14] = (uint16_t)((d[19] >> 2 | d[20] << 6))                 & 0x07FFU;
    s_channels[15] = (uint16_t)((d[20] >> 5 | d[21] << 3))                 & 0x07FFU;

    uint8_t flags = f[23];
    s_failsafe = (flags & (SBUS_FLAG_FRAME_LOST | SBUS_FLAG_FAILSAFE)) != 0U;
    s_hasFrame = true;
    s_lastFrameTick = HAL_GetTick();
}

void SBUS_Init(void)
{
    s_rxIndex = 0U;
    s_hasFrame = false;
    s_failsafe = true; /* no frame yet -> treat as failsafe until proven otherwise */
    s_lastFrameTick = 0U;
    for (uint8_t i = 0U; i < SBUS_NUM_CHANNELS; i++) {
        s_channels[i] = SBUS_CH_MID;
    }
    HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
}

uint16_t SBUS_GetChannel(uint8_t idx)
{
    if (idx >= SBUS_NUM_CHANNELS) {
        return SBUS_CH_MID;
    }
    return s_channels[idx];
}

bool SBUS_HasFrame(void)
{
    return s_hasFrame;
}

bool SBUS_IsFailsafe(void)
{
    return s_failsafe;
}

bool SBUS_IsLinkUp(void)
{
    if (!s_hasFrame || s_failsafe) {
        return false;
    }
    return (HAL_GetTick() - s_lastFrameTick) <= SBUS_LINK_TIMEOUT_MS;
}

void SBUS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    if (s_rxIndex == 0U && s_rxByte != SBUS_START_BYTE) {
        /* not synced yet; keep hunting for the start byte */
        HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
        return;
    }

    s_rxFrame[s_rxIndex] = s_rxByte;
    s_rxIndex++;

    if (s_rxIndex >= SBUS_FRAME_LEN) {
        if (s_rxFrame[SBUS_FRAME_LEN - 1U] == SBUS_END_BYTE) {
            DecodeFrame(s_rxFrame);
        }
        s_rxIndex = 0U;
    }

    HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
}

void SBUS_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }
    s_rxIndex = 0U;
    HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
}
