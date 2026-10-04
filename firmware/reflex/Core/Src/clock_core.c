#include "clock.h"

void ClockCore_Update(ClockCore *c, uint32_t cycles32)
{
    if (!c->initialised) {
        c->last_cycles = cycles32;
        c->total_cycles = 0U;
        c->initialised = 1;
        return;
    }
    c->total_cycles += (uint32_t)(cycles32 - c->last_cycles);
    c->last_cycles = cycles32;
}

uint64_t ClockCore_Us(const ClockCore *c, uint32_t cycles_per_us)
{
    if (cycles_per_us == 0U) {
        return 0U;
    }
    return c->total_cycles / cycles_per_us;
}
