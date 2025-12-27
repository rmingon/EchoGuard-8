#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GNSS_MODULE_COUNT
#define GNSS_MODULE_COUNT 8u
#endif

typedef struct {
	uint8_t module_index; /* 1..8 */

	bool has_fix;
	uint8_t fix_quality;
	uint8_t satellites;
	uint16_t hdop_centi;

	int32_t lat_e7;
	int32_t lon_e7;
	int32_t alt_cm;

	uint16_t speed_centi_ms;
	uint16_t course_centi_deg;

	uint32_t last_fix_tick;

	uint32_t nmea_sentences;
	uint32_t nmea_checksum_errors;
} GnssModuleState;

void Gnss_Init(uint32_t baudrate);
void Gnss_Task(void *argument);
const GnssModuleState *Gnss_GetModules(void);
const GnssModuleState *Gnss_GetModule(uint8_t module_index);

#ifdef __cplusplus
}
#endif
