#ifndef ESC_H
#define ESC_H

#include <stdint.h>

/* Standard bidirectional RC ESC pulse range, 50 Hz frame (TIM2 configured
 * for 1 us/tick, 20 ms period). 1500 us is the stopped/neutral position. */
#define ESC_PULSE_MIN_US     1000U
#define ESC_PULSE_NEUTRAL_US 1500U
#define ESC_PULSE_MAX_US     2000U

typedef enum {
    ESC_MOTOR_1 = 0, /* TIM2_CH1 / PA15 */
    ESC_MOTOR_2 = 1, /* TIM2_CH2 / PA1  */
} ESC_Motor;

/* Starts PWM on both channels and immediately commands neutral (stop) —
 * must be called before anything else can command a non-neutral pulse. */
void ESC_Init(void);

/* Clamps to [ESC_PULSE_MIN_US, ESC_PULSE_MAX_US] before writing. */
void ESC_SetPulse(ESC_Motor motor, uint16_t pulse_us);

void ESC_Stop(void);

#endif /* ESC_H */
