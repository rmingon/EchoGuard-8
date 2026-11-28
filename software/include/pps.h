#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 1PPS of GNSS module #6 (U12) on PA1, captured by TIM2_CH2 on the rising edge (aligned with
 * the UTC second). TIM2 runs at 1 MHz and is extended to 32 bits in software, so timestamps
 * are microseconds of the local clock (wraps after ~71 min, compare with unsigned differences).
 */
typedef struct {
	uint32_t count;      /* Edges captured since boot. */
	uint32_t last_us;    /* Local timestamp of the last edge. */
	uint32_t last_tick;  /* HAL tick (ms) of the last edge. */
	uint32_t period_us;  /* Local duration of the last full second, 0 if unknown. */
} PpsInfo;

/* A second measured shorter/longer than this is a glitch or a missed pulse, not a period. */
#define PPS_PERIOD_MIN_US 900000u
#define PPS_PERIOD_MAX_US 1100000u

void Pps_Init(void);
void Pps_TimIrqHandler(void);

/* Current local time on the PPS timebase. */
uint32_t Pps_NowUs(void);

/* Copy of the last capture. Returns false if no edge was seen in the last 1.5 s. */
bool Pps_GetLast(PpsInfo *out);

#ifdef __cplusplus
}
#endif
