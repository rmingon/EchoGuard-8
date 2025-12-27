#include "gnss.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "gnss_uart.h"
#include "nmea.h"

typedef struct {
	char line[96];
	uint16_t used;
} LineBuffer;

static GnssModuleState modules[GNSS_MODULE_COUNT];
static LineBuffer line_buffers[GNSS_MODULE_COUNT];

static GnssModuleState *module_by_index(uint8_t module_index) {
	if (module_index < 1 || module_index > GNSS_MODULE_COUNT) {
		return NULL;
	}
	return &modules[module_index - 1];
}

static LineBuffer *line_by_index(uint8_t module_index) {
	if (module_index < 1 || module_index > GNSS_MODULE_COUNT) {
		return NULL;
	}
	return &line_buffers[module_index - 1];
}

const GnssModuleState *Gnss_GetModules(void) {
	return modules;
}

const GnssModuleState *Gnss_GetModule(uint8_t module_index) {
	return module_by_index(module_index);
}

void Gnss_Init(uint32_t baudrate) {
	for (uint8_t i = 1; i <= GNSS_MODULE_COUNT; i++) {
		GnssModuleState *m = module_by_index(i);
		LineBuffer *lb = line_by_index(i);
		if (m != NULL) {
			memset(m, 0, sizeof(*m));
			m->module_index = i;
		}
		if (lb != NULL) {
			memset(lb, 0, sizeof(*lb));
		}
	}

	GnssUart_GpioInit();
	GnssUart_HardwareUartsInit(baudrate);
	GnssUart_SoftUartInit(baudrate);
	GnssUart_StartHardwareRx();
}

static void ingest_line(uint8_t module_index, const char *line) {
	GnssModuleState *m = module_by_index(module_index);
	if (m == NULL) {
		return;
	}
	m->nmea_sentences++;

	if (!Nmea_ChecksumOk(line)) {
		m->nmea_checksum_errors++;
		return;
	}

	NmeaGga gga = {0};
	if (Nmea_ParseGga(line, &gga)) {
		m->has_fix = (gga.fix_quality > 0);
		m->fix_quality = gga.fix_quality;
		m->satellites = gga.satellites;
		m->hdop_centi = gga.hdop_centi;
		m->lat_e7 = gga.lat_e7;
		m->lon_e7 = gga.lon_e7;
		m->alt_cm = gga.alt_cm;
		m->last_fix_tick = HAL_GetTick();
		return;
	}

	NmeaRmc rmc = {0};
	if (Nmea_ParseRmc(line, &rmc)) {
		if (rmc.status == (uint8_t)'A') {
			m->has_fix = true;
			m->lat_e7 = rmc.lat_e7;
			m->lon_e7 = rmc.lon_e7;
			m->speed_centi_ms = rmc.speed_centi_ms;
			m->course_centi_deg = rmc.course_centi_deg;
			m->last_fix_tick = HAL_GetTick();
		}
		return;
	}
}

static void ingest_bytes(uint8_t module_index, const uint8_t *data, size_t len) {
	LineBuffer *lb = line_by_index(module_index);
	if (lb == NULL) {
		return;
	}

	for (size_t i = 0; i < len; i++) {
		char c = (char)data[i];
		if (c == '\r') {
			continue;
		}
		if (c == '\n') {
			if (lb->used > 0) {
				lb->line[lb->used] = '\0';
				ingest_line(module_index, lb->line);
			}
			lb->used = 0;
			continue;
		}
		if (lb->used + 1u >= sizeof(lb->line)) {
			lb->used = 0;
			continue;
		}
		lb->line[lb->used++] = c;
	}
}

void Gnss_Task(void *argument) {
	(void)argument;

	uint8_t scratch[64];

	while (1) {
		for (uint8_t module_index = 1; module_index <= GNSS_MODULE_COUNT; module_index++) {
			size_t n = GnssUart_ReadBytes(module_index, scratch, sizeof(scratch));
			if (n > 0) {
				ingest_bytes(module_index, scratch, n);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
