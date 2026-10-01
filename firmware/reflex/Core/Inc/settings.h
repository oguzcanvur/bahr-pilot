#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include "pi_link.h"

/* Persists the RC map (RCMAP_x / RCn_MIN/MAX/TRIM/REVERSED, see pi_link.h)
 * across power cycles by writing it to the last 2 KB flash page
 * (STM32G431RB: 128 KB flash, 64 pages of 2 KB, page 63 at
 * FLASH_BASE + 63*2KB = 0x0801F800). Not reserved in the linker script —
 * the app image is ~20 KB, nowhere near that address, so this is the
 * common "steal the last page for settings" pattern rather than a real
 * memory-map allocation. If the app ever grows close to 126 KB this would
 * need revisiting (it won't: there's no header/table describing it, and
 * the app would silently overwrite it instead of erroring). */

void Settings_Init(void);

/* Loads the saved RC map into *out. Returns false (and leaves *out
 * untouched) if flash is blank or the stored record fails its checksum —
 * callers should keep their compiled-in defaults in that case. */
bool Settings_LoadRcMap(RcMapConfig *out);

/* Erases the settings page and writes the current RC map. Takes a few
 * milliseconds (flash erase/program) — only called when a config frame
 * actually changes the mapping, not from the periodic control loop. */
void Settings_SaveRcMap(const RcMapConfig *map);

#endif /* SETTINGS_H */
