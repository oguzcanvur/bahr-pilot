#include "settings.h"
#include "main.h"
#include <string.h>

/* On-flash record layout, 32 bytes (4 double-words — STM32G4 flash is
 * programmed 64 bits at a time):
 *   [0..3]   magic ("BAH2" sentinel; also tells a real record from blank/
 *            erased flash, which reads back as 0xFF, and rejects records
 *            written by the earlier 24-byte layout)
 *   [4]      throttle_channel  [5] steering_channel
 *   [6]      arm_channel       [7] mode_channel
 *   [8..9]   throttle_min      [10..11] throttle_max    [12..13] throttle_trim
 *   [14..15] steering_min      [16..17] steering_max     [18..19] steering_trim
 *   [20]     flags (bit0=throttle_reversed, bit1=steering_reversed)
 *   [21]     manual_slot_mask
 *   [22..23] mode_min          [24..25] mode_max
 *   [26]     XOR checksum of bytes 0-25
 *   [27..31] pad (0), rounds the record to a double-word multiple
 * All multi-byte fields are native (little-endian on Cortex-M4) — this is
 * read back by the same firmware image that wrote it, never by another
 * machine, so there's no cross-platform endianness concern. */

#define RECORD_LEN      32U
#define RECORD_BODY_LEN 26U
#define RECORD_MAGIC    0x32484142UL /* "BAH2", little-endian bytes 'B','A','H','2' */

static uint32_t SettingsAddress(void)
{
    /* FLASH_PAGE_NB/FLASH_SIZE (stm32g4xx_hal_flash.h) resolve at runtime
     * from the chip's own flash-size register, not a compile-time literal
     * — safe to use in ordinary arithmetic. Last page on this part: 128 KB
     * flash, 64 pages of 2 KB, so page 63 at 0x0801F800. */
    return FLASH_BASE + (FLASH_PAGE_NB - 1U) * FLASH_PAGE_SIZE;
}

static uint8_t Checksum(const uint8_t *data, uint32_t len)
{
    uint8_t x = 0U;
    for (uint32_t i = 0U; i < len; i++) {
        x ^= data[i];
    }
    return x;
}

static void Serialize(const RcMapConfig *map, uint8_t *buf)
{
    memset(buf, 0, RECORD_LEN);
    uint32_t magic = RECORD_MAGIC;
    memcpy(&buf[0], &magic, 4U);
    buf[4] = map->throttle_channel;
    buf[5] = map->steering_channel;
    buf[6] = map->arm_channel;
    buf[7] = map->mode_channel;
    memcpy(&buf[8], &map->throttle_min, 2U);
    memcpy(&buf[10], &map->throttle_max, 2U);
    memcpy(&buf[12], &map->throttle_trim, 2U);
    memcpy(&buf[14], &map->steering_min, 2U);
    memcpy(&buf[16], &map->steering_max, 2U);
    memcpy(&buf[18], &map->steering_trim, 2U);
    buf[20] = (uint8_t)((map->throttle_reversed ? 0x01U : 0U) | (map->steering_reversed ? 0x02U : 0U));
    buf[21] = map->manual_slot_mask;
    memcpy(&buf[22], &map->mode_min, 2U);
    memcpy(&buf[24], &map->mode_max, 2U);
    buf[26] = Checksum(buf, RECORD_BODY_LEN);
}

void Settings_Init(void)
{
    /* Nothing to do — flash needs no setup beyond HAL_Init(), which has
     * already run by the time anyone calls Settings_LoadRcMap(). Kept as
     * a function for symmetry with the rest of this project's *_Init()
     * modules and in case that changes later. */
}

bool Settings_LoadRcMap(RcMapConfig *out)
{
    const uint8_t *flash = (const uint8_t *)SettingsAddress();
    uint32_t magic;
    memcpy(&magic, &flash[0], 4U);
    if (magic != RECORD_MAGIC) {
        return false; /* blank (erased = 0xFF...) or never written */
    }
    if (Checksum(flash, RECORD_BODY_LEN) != flash[26]) {
        return false; /* torn/corrupt write (e.g. power loss mid-erase) */
    }
    /* A valid checksum only proves the bytes are intact, not that they make
     * sense — never let a bad channel index through to SBUS_GetChannel(). */
    if (flash[4] >= SBUS_NUM_CHANNELS || flash[5] >= SBUS_NUM_CHANNELS ||
        flash[6] >= SBUS_NUM_CHANNELS || flash[7] >= SBUS_NUM_CHANNELS) {
        return false;
    }

    out->throttle_channel = flash[4];
    out->steering_channel = flash[5];
    out->arm_channel = flash[6];
    out->mode_channel = flash[7];
    memcpy(&out->throttle_min, &flash[8], 2U);
    memcpy(&out->throttle_max, &flash[10], 2U);
    memcpy(&out->throttle_trim, &flash[12], 2U);
    memcpy(&out->steering_min, &flash[14], 2U);
    memcpy(&out->steering_max, &flash[16], 2U);
    memcpy(&out->steering_trim, &flash[18], 2U);
    out->throttle_reversed = (flash[20] & 0x01U) != 0U;
    out->steering_reversed = (flash[20] & 0x02U) != 0U;
    out->manual_slot_mask = (uint8_t)(flash[21] & 0x3FU);
    memcpy(&out->mode_min, &flash[22], 2U);
    memcpy(&out->mode_max, &flash[24], 2U);
    return true;
}

void Settings_SaveRcMap(const RcMapConfig *map)
{
    uint8_t buf[RECORD_LEN];
    Serialize(map, buf);

    uint32_t addr = SettingsAddress();

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = FLASH_PAGE_NB - 1U;
    erase.NbPages = 1U;
    uint32_t pageError = 0U;
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    for (uint32_t offset = 0U; offset < RECORD_LEN; offset += 8U) {
        uint64_t doubleWord;
        memcpy(&doubleWord, &buf[offset], 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, doubleWord) != HAL_OK) {
            break; /* leaves a torn record — the checksum on next boot's
                     * Settings_LoadRcMap() catches this and falls back to
                     * compiled defaults rather than loading garbage */
        }
    }

    HAL_FLASH_Lock();
}
