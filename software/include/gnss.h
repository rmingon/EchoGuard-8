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

typedef enum {
	GNSS_CONST_GPS = 0,
	GNSS_CONST_BDS,
	GNSS_CONST_GLO,
	GNSS_CONST_OTHER,
	GNSS_CONST_COUNT,
} GnssConstellation;

#define GNSS_CN0_TOP_COUNT 4u

/* A constellation whose last GSV cycle is older than this is ignored (lost or disabled). */
#define GNSS_CN0_STALE_MS 3000u

/* Signal strength of one constellation, taken from its last complete GSV cycle. */
typedef struct {
	uint8_t in_view;
	uint8_t tracked; /* Satellites with a C/N0 value. */
	uint16_t cn0_sum; /* dB-Hz, over tracked satellites. */
	uint8_t cn0_top[GNSS_CN0_TOP_COUNT]; /* Strongest C/N0, descending, 0 = unused. */
	uint32_t update_tick; /* HAL tick of the last complete cycle (0 = never). */
} GnssCn0Constellation;

/* C/N0 of one module, merged over its constellations with fresh data. */
typedef struct {
	bool valid; /* At least one constellation reported a complete GSV cycle recently. */
	uint8_t in_view;
	uint8_t tracked;
	uint8_t max_dbhz;
	uint16_t mean_x10;  /* Mean C/N0 of tracked satellites, dB-Hz x10. */
	uint16_t top4_x10;  /* Mean of the 4 strongest satellites, dB-Hz x10 (jamming indicator). */
} GnssCn0Summary;

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
	uint32_t fix_time_ms; /* UTC ms of day of the lat/lon above (GNSS epoch). */
	uint32_t last_rx_tick; /* Last NMEA sentence with a valid checksum (0 = never). */

	uint32_t nmea_sentences;
	uint32_t nmea_checksum_errors;

	GnssCn0Constellation cn0[GNSS_CONST_COUNT];
} GnssModuleState;

/* Low time of the shared MOD_RST pulse (the datasheet gives no minimum). */
#define GNSS_RESET_PULSE_MS 10u

/* Upper bound for one command; a 40-byte frame takes ~42 ms at 9600 baud. */
#define GNSS_COMMAND_TIMEOUT_MS 200u

void Gnss_Init(uint32_t baudrate);

/*
 * Pulse the shared nRESET line of all 8 modules. Blocks for GNSS_RESET_PULSE_MS; the modules
 * then reboot and print their $GPTXT banner before sending NMEA again.
 */
void Gnss_ResetModules(void);

/* ISR-safe: ask Gnss_Task to run Gnss_ResetModules() on its next pass (within ~5 ms). */
void Gnss_RequestReset(void);

/*
 * Send one NMEA/CASIC command to a module (1..8). body is the text between '$' and '*'
 * (e.g. "PCAS10,0" for a hot start); the checksum and CRLF are added. Blocks until sent.
 */
bool Gnss_SendCommand(uint8_t module_index, const char *body);

/* Same command to every module, one after the other. Returns how many were sent. */
uint8_t Gnss_SendCommandAll(const char *body);
void Gnss_Task(void *argument);
const GnssModuleState *Gnss_GetModules(void);
const GnssModuleState *Gnss_GetModule(uint8_t module_index);

/* Merge the constellations updated within GNSS_CN0_STALE_MS. Returns out->valid. */
bool Gnss_GetCn0Summary(const GnssModuleState *m, uint32_t now, GnssCn0Summary *out);

#ifdef __cplusplus
}
#endif
