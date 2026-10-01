#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

#define SBUS_NUM_CHANNELS 16

/* Raw 11-bit SBUS channel range (FrSky/Futaba convention). */
#define SBUS_CH_MIN 172U
#define SBUS_CH_MID 992U
#define SBUS_CH_MAX 1811U

/* RC signal is considered gone if no valid frame arrived within this long —
 * backs up the receiver's own failsafe flag in case the receiver itself
 * locks up instead of reporting failsafe. */
#define SBUS_LINK_TIMEOUT_MS 200U

void SBUS_Init(void);

/* Raw 11-bit value for channel idx (0-15). Returns SBUS_CH_MID if idx is
 * out of range or no frame has been received yet. */
uint16_t SBUS_GetChannel(uint8_t idx);

/* True once a well-formed frame has been received and parsed. */
bool SBUS_HasFrame(void);

/* True if the receiver itself reports failsafe (transmitter signal lost,
 * receiver fell back to failsafe channel values) or frame-lost. */
bool SBUS_IsFailsafe(void);

/* True if a valid frame arrived within SBUS_LINK_TIMEOUT_MS and the
 * receiver isn't reporting failsafe — the single call failsafe logic
 * should use to decide "is the RC link actually up". */
bool SBUS_IsLinkUp(void);

/* Must be called from HAL_UART_RxCpltCallback for USART1; ignored if
 * huart isn't USART1. */
void SBUS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* Must be called from HAL_UART_ErrorCallback for USART1 to re-arm
 * reception after a framing/noise/overrun error. */
void SBUS_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* SBUS_H */
