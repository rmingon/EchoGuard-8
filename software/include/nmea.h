#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t time_ms_of_day;
	int32_t lat_e7;
	int32_t lon_e7;
	int32_t alt_cm;
	uint16_t hdop_centi;
	uint8_t fix_quality;
	uint8_t satellites;
} NmeaGga;

typedef struct {
	uint32_t time_ms_of_day;
	int32_t lat_e7;
	int32_t lon_e7;
	uint8_t status;
	uint16_t speed_centi_ms;
	uint16_t course_centi_deg;
} NmeaRmc;

bool Nmea_ChecksumOk(const char *sentence);
bool Nmea_ParseGga(const char *sentence, NmeaGga *out);
bool Nmea_ParseRmc(const char *sentence, NmeaRmc *out);

#ifdef __cplusplus
}
#endif

