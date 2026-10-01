#include "esc.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

static uint16_t ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < ESC_PULSE_MIN_US) {
        return ESC_PULSE_MIN_US;
    }
    if (pulse_us > ESC_PULSE_MAX_US) {
        return ESC_PULSE_MAX_US;
    }
    return pulse_us;
}

void ESC_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    ESC_Stop();
}

void ESC_SetPulse(ESC_Motor motor, uint16_t pulse_us)
{
    pulse_us = ClampPulse(pulse_us);
    uint32_t channel = (motor == ESC_MOTOR_1) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
    __HAL_TIM_SET_COMPARE(&htim2, channel, pulse_us);
}

void ESC_Stop(void)
{
    ESC_SetPulse(ESC_MOTOR_1, ESC_PULSE_NEUTRAL_US);
    ESC_SetPulse(ESC_MOTOR_2, ESC_PULSE_NEUTRAL_US);
}
