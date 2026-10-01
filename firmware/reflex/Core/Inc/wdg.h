#ifndef WDG_H
#define WDG_H

/* Independent watchdog (IWDG), driven by direct register access rather
 * than HAL_IWDG — the HAL IWDG driver source isn't part of this project
 * (CubeMX never generated it since IWDG wasn't enabled in reflex.ioc, and
 * adding it would mean hand-editing the Eclipse-generated build file list;
 * IWDG itself is 4 registers, simple enough to not need the abstraction).
 * Catches firmware lockups (failsafe table priority 5): if Wdg_Refresh()
 * isn't called often enough, the MCU resets itself and reboots into the
 * disarmed default-stop state. */

void Wdg_Init(void);
void Wdg_Refresh(void);

#endif /* WDG_H */
