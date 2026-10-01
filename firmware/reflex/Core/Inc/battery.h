#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

/* ADC1_IN1 / PA0, through an external resistor divider (board has no ADC
 * input that tolerates LiPo pack voltage directly). Call Battery_Update()
 * periodically (main loop) and read the filtered result with
 * Battery_GetMilliVolts(). */

void Battery_Init(void);
void Battery_Update(void);

/* Filtered pack voltage in millivolts. 0 until the first conversion
 * completes (Battery_IsValid() false until then). */
uint16_t Battery_GetMilliVolts(void);
bool Battery_IsValid(void);

#endif /* BATTERY_H */
