#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	GNSS_FUSION_NO_FIX = 0,
	GNSS_FUSION_OK = 1,
	GNSS_FUSION_DEGRADED = 2,
	GNSS_FUSION_INTERFERENCE = 3,
} GnssFusionStatus;

typedef struct {
	bool has_fix;
	GnssFusionStatus status;

	int32_t lat_e7;
	int32_t lon_e7;
	int32_t alt_cm;

	uint8_t used_modules;
	uint8_t rejected_modules;

	uint16_t max_residual_cm;
	uint16_t avg_hdop_centi;

	uint32_t last_update_tick;
} GnssFusionResult;

void GnssFusion_Init(void);
void GnssFusion_Task(void *argument);
bool GnssFusion_GetResult(GnssFusionResult *out);
bool GnssFusion_GetModuleFaultScore(uint8_t module_index, uint16_t *out_score);

#ifdef __cplusplus
}
#endif
