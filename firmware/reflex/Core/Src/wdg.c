#include "wdg.h"
#include "main.h"

/* Raw register access (IWDG->KR/PR/RLR/SR come from the CMSIS device
 * header, always present regardless of HAL_IWDG_MODULE_ENABLED) rather
 * than HAL_IWDG_Init/Refresh — see wdg.h for why.
 *
 * LSI ~32 kHz (independent of the main HSI/PLL clock, so this still fires
 * even if the system clock or a peripheral clock has wedged).
 * Prescaler /64 -> 2 ms/tick. Reload 249 -> (249+1)*2ms = 500 ms timeout,
 * matching PI_LINK_TIMEOUT_MS/SBUS_LINK_TIMEOUT_MS's own 500/200 ms scale
 * for "something has gone quiet" in this project. */
#define IWDG_KEY_ENABLE        0xCCCCU
#define IWDG_KEY_WRITE_ACCESS  0x5555U
#define IWDG_KEY_REFRESH       0xAAAAU
#define IWDG_PRESCALER_64_CODE 0x04U
#define IWDG_RELOAD_500MS      249U

void Wdg_Init(void)
{
    IWDG->KR = IWDG_KEY_ENABLE;
    IWDG->KR = IWDG_KEY_WRITE_ACCESS;
    IWDG->PR = IWDG_PRESCALER_64_CODE;
    IWDG->RLR = IWDG_RELOAD_500MS;
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) {
        /* wait for the prescaler/reload write to land in the LSI clock domain */
    }
    IWDG->KR = IWDG_KEY_REFRESH;
}

void Wdg_Refresh(void)
{
    IWDG->KR = IWDG_KEY_REFRESH;
}
