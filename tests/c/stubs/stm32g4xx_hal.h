/* Host-test stand-in for the real stm32g4xx_hal.h: just enough of the HAL
 * surface for pi_link.c to compile on a PC. Put first on the include path
 * (-I tests/c/stubs), so it shadows the real header. Never used for the
 * firmware build. */
#ifndef STUB_STM32G4XX_HAL_H
#define STUB_STM32G4XX_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    void *Instance;
} UART_HandleTypeDef;

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;

#define USART3 ((void *)3)

#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#endif /* STUB_STM32G4XX_HAL_H */
