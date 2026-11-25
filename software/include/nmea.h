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

#define NMEA_GSV_SATS_PER_MSG 4u

typedef struct {
	uint8_t prn;
	uint8_t elevation_deg; /* 0..90, 0 when the field is empty. */
	uint8_t cn0_dbhz;      /* 0 = satellite in view but not tracked (empty SNR field). */
} NmeaGsvSat;

/* One $xxGSV sentence: a page of the satellites-in-view list of one constellation. */
typedef struct {
	char talker[2]; /* "GP", "BD", "GB", "GL", "GA", ... */
	uint8_t total_msgs;
	uint8_t msg_num; /* 1..total_msgs */
	uint8_t sats_in_view;
	uint8_t sat_count; /* Entries used in sats[]. */
	NmeaGsvSat sats[NMEA_GSV_SATS_PER_MSG];
} NmeaGsv;

bool Nmea_ChecksumOk(const char *sentence);
bool Nmea_ParseGga(const char *sentence, NmeaGga *out);
bool Nmea_ParseRmc(const char *sentence, NmeaRmc *out);
bool Nmea_ParseGsv(const char *sentence, NmeaGsv *out);

#ifdef __cplusplus
}
#endif

