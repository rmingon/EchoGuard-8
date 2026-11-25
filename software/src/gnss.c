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

/* GSV cycle being assembled for one constellation of one module. */
typedef struct {
	uint8_t next_msg; /* Page expected next, 0 = waiting for page 1. */
	uint8_t total_msgs;
	GnssCn0Constellation work;
} GsvAccumulator;

static GnssModuleState modules[GNSS_MODULE_COUNT];
static LineBuffer line_buffers[GNSS_MODULE_COUNT];
static GsvAccumulator gsv_acc[GNSS_MODULE_COUNT][GNSS_CONST_COUNT];

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
	memset(gsv_acc, 0, sizeof(gsv_acc));

	GnssUart_GpioInit();
	GnssUart_HardwareUartsInit(baudrate);
	GnssUart_SoftUartInit(baudrate);
	GnssUart_StartHardwareRx();
}

static GnssConstellation constellation_from_talker(const char talker[2]) {
	if (talker[0] == 'G' && talker[1] == 'P') {
		return GNSS_CONST_GPS;
	}
	if ((talker[0] == 'B' && talker[1] == 'D') || (talker[0] == 'G' && talker[1] == 'B')) {
		return GNSS_CONST_BDS;
	}
	if (talker[0] == 'G' && talker[1] == 'L') {
		return GNSS_CONST_GLO;
	}
	return GNSS_CONST_OTHER;
}

/* Insert into a descending array of size GNSS_CN0_TOP_COUNT, dropping the weakest. */
static void top_insert(uint8_t *top, uint8_t value) {
	for (size_t i = 0; i < GNSS_CN0_TOP_COUNT; i++) {
		if (value > top[i]) {
			for (size_t j = GNSS_CN0_TOP_COUNT - 1u; j > i; j--) {
				top[j] = top[j - 1u];
			}
			top[i] = value;
			return;
		}
	}
}

static void ingest_gsv(uint8_t module_index, GnssModuleState *m, const NmeaGsv *gsv) {
	GnssConstellation c = constellation_from_talker(gsv->talker);
	GsvAccumulator *acc = &gsv_acc[module_index - 1][c];

	if (gsv->msg_num == 1) {
		memset(&acc->work, 0, sizeof(acc->work));
		acc->total_msgs = gsv->total_msgs;
		acc->next_msg = 1;
	}
	/* A page lost to a checksum error would bias the cycle: drop it and wait for the next one. */
	if (acc->next_msg != gsv->msg_num || acc->total_msgs != gsv->total_msgs) {
		acc->next_msg = 0;
		return;
	}

	acc->work.in_view = gsv->sats_in_view;
	for (uint8_t i = 0; i < gsv->sat_count; i++) {
		uint8_t cn0 = gsv->sats[i].cn0_dbhz;
		if (cn0 == 0) {
			continue;
		}
		acc->work.tracked++;
		acc->work.cn0_sum = (uint16_t)(acc->work.cn0_sum + cn0);
		top_insert(acc->work.cn0_top, cn0);
	}

	if (gsv->msg_num == gsv->total_msgs) {
		acc->work.update_tick = HAL_GetTick();
		m->cn0[c] = acc->work;
		acc->next_msg = 0;
	} else {
		acc->next_msg++;
	}
}

bool Gnss_GetCn0Summary(const GnssModuleState *m, uint32_t now, GnssCn0Summary *out) {
	memset(out, 0, sizeof(*out));
	if (m == NULL) {
		return false;
	}

	uint8_t top[GNSS_CN0_TOP_COUNT] = {0};
	uint32_t sum = 0;
	for (size_t c = 0; c < GNSS_CONST_COUNT; c++) {
		const GnssCn0Constellation *cc = &m->cn0[c];
		if (cc->update_tick == 0 || (now - cc->update_tick) > GNSS_CN0_STALE_MS) {
			continue;
		}
		out->valid = true;
		out->in_view = (uint8_t)(out->in_view + cc->in_view);
		out->tracked = (uint8_t)(out->tracked + cc->tracked);
		sum += cc->cn0_sum;
		for (size_t i = 0; i < GNSS_CN0_TOP_COUNT && cc->cn0_top[i] != 0; i++) {
			top_insert(top, cc->cn0_top[i]);
		}
	}

	if (out->tracked > 0) {
		out->mean_x10 = (uint16_t)((sum * 10u) / out->tracked);
		out->max_dbhz = top[0];
		uint32_t top_sum = 0;
		uint32_t top_n = 0;
		for (size_t i = 0; i < GNSS_CN0_TOP_COUNT && top[i] != 0; i++) {
			top_sum += top[i];
			top_n++;
		}
		out->top4_x10 = (uint16_t)((top_sum * 10u) / top_n);
	}
	return out->valid;
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
	m->last_rx_tick = HAL_GetTick();

	NmeaGga gga = {0};
	if (Nmea_ParseGga(line, &gga)) {
		m->has_fix = (gga.fix_quality > 0);
		m->fix_quality = gga.fix_quality;
		m->satellites = gga.satellites;
		m->hdop_centi = gga.hdop_centi;
		m->lat_e7 = gga.lat_e7;
		m->lon_e7 = gga.lon_e7;
		m->alt_cm = gga.alt_cm;
		m->fix_time_ms = gga.time_ms_of_day;
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
			m->fix_time_ms = rmc.time_ms_of_day;
			m->last_fix_tick = HAL_GetTick();
		}
		return;
	}

	NmeaGsv gsv = {0};
	if (Nmea_ParseGsv(line, &gsv)) {
		ingest_gsv(module_index, m, &gsv);
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
