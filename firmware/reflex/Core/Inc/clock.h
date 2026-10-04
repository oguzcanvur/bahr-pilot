#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

/* Microsecond time base (Phase 3). Every sensor measurement that leaves the
 * STM carries a timestamp from this clock, so the Pi can align IMU, GNSS and
 * motor events.
 *
 * Source: the Cortex-M4 DWT cycle counter (170 MHz -> ~5.9 ns resolution).
 * It is only 32 bits and wraps every ~25 s, so ClockCore extends it to 64
 * bits; that needs one call at least every ~12 s, which the main loop makes
 * thousands of times per second.
 *
 * Accuracy: the system clock runs from the internal HSI oscillator (no
 * crystal on this board), good to roughly +-1 %, so "a microsecond" here may
 * be 0.99-1.01 real microseconds. Intervals are therefore only as good as
 * that — the Pi estimates and removes the drift (bahr_pilot/timesync.py)
 * instead of trusting this clock to be absolute. */

/* --- pure part (no HAL), tested on the host: tests/c/test_clock_core.c --- */

typedef struct {
    uint32_t last_cycles;
    uint64_t total_cycles;
    int initialised;
} ClockCore;

/* Folds a new 32-bit counter reading into the running 64-bit total.
 * Unsigned subtraction makes the wrap harmless as long as less than one
 * full wrap (2^32 cycles) passed since the previous call. */
void ClockCore_Update(ClockCore *c, uint32_t cycles32);

/* Microseconds since the first update, given the core clock in cycles/us. */
uint64_t ClockCore_Us(const ClockCore *c, uint32_t cycles_per_us);

/* --- hardware part ------------------------------------------------------ */

void Clock_Init(void);

/* Monotonic microseconds since Clock_Init(). Call from the main loop (not an
 * ISR that can preempt another caller): it updates shared state. */
uint64_t Clock_NowUs(void);

/* Low 32 bits of Clock_NowUs(), the form that goes on the wire (wraps every
 * 71.6 minutes; the Pi unwraps it). */
uint32_t Clock_NowUs32(void);

#endif /* CLOCK_H */
