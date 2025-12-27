#include "gnss_fusion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "gnss.h"

static GnssFusionResult latest;
static uint16_t fault_score[GNSS_MODULE_COUNT];

static void sort_i32(int32_t *values, size_t count) {
	for (size_t i = 1; i < count; i++) {
		int32_t v = values[i];
		size_t j = i;
		while (j > 0 && values[j - 1] > v) {
			values[j] = values[j - 1];
			j--;
		}
		values[j] = v;
	}
}

static int32_t median_i32(int32_t *values, size_t count) {
	sort_i32(values, count);
	return values[count / 2];
}

void GnssFusion_Init(void) {
	memset(&latest, 0, sizeof(latest));
	memset(fault_score, 0, sizeof(fault_score));
}

static float clampf(float v, float lo, float hi) {
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

static uint16_t clamp_u16(uint32_t v) {
	return (uint16_t)(v > 65535u ? 65535u : v);
}

static void update_fault_score(uint8_t module_index, bool used, bool rejected, float residual_m,
                               float threshold_m) {
	if (module_index < 1 || module_index > GNSS_MODULE_COUNT) {
		return;
	}
	uint16_t *score = &fault_score[module_index - 1];

	if (rejected) {
		uint32_t next = (uint32_t)(*score) + 3u;
		*score = (uint16_t)(next > 500u ? 500u : next);
		return;
	}
	if (!used) {
		uint32_t next = (uint32_t)(*score) + 1u;
		*score = (uint16_t)(next > 500u ? 500u : next);
		return;
	}

	if (residual_m < (threshold_m * 0.5f)) {
		*score = (uint16_t)(*score > 2u ? (*score - 2u) : 0u);
	} else if (residual_m > threshold_m) {
		uint32_t next = (uint32_t)(*score) + 2u;
		*score = (uint16_t)(next > 500u ? 500u : next);
	} else {
		*score = (uint16_t)(*score > 1u ? (*score - 1u) : 0u);
	}
}

static void compute_fusion(void) {
	const GnssModuleState *modules = Gnss_GetModules();

	uint32_t now = HAL_GetTick();

	const GnssModuleState *candidates[GNSS_MODULE_COUNT] = {0};
	size_t candidate_count = 0;
	for (size_t i = 0; i < GNSS_MODULE_COUNT; i++) {
		const GnssModuleState *m = &modules[i];
		if (!m->has_fix || m->fix_quality == 0 || m->hdop_centi == 0) {
			continue;
		}
		if ((now - m->last_fix_tick) > 2000u) {
			continue;
		}
		if (fault_score[i] >= 100u) {
			continue;
		}
		candidates[candidate_count++] = m;
	}

	GnssFusionResult r = {0};
	r.last_update_tick = now;

	if (candidate_count == 0) {
		r.status = GNSS_FUSION_NO_FIX;
		taskENTER_CRITICAL();
		latest = r;
		taskEXIT_CRITICAL();
		return;
	}

	int32_t lat_buf[GNSS_MODULE_COUNT];
	int32_t lon_buf[GNSS_MODULE_COUNT];
	for (size_t i = 0; i < candidate_count; i++) {
		lat_buf[i] = candidates[i]->lat_e7;
		lon_buf[i] = candidates[i]->lon_e7;
	}

	int32_t med_lat_e7 = median_i32(lat_buf, candidate_count);
	int32_t med_lon_e7 = median_i32(lon_buf, candidate_count);
	float med_lat_deg = (float)med_lat_e7 * 1e-7f;
	float lat_rad = med_lat_deg * (float)(3.14159265358979323846 / 180.0);
	float meters_per_deg_lon = 111320.0f * cosf(lat_rad);

	const GnssModuleState *used[GNSS_MODULE_COUNT] = {0};
	float used_weights[GNSS_MODULE_COUNT] = {0};
	float used_residual_m[GNSS_MODULE_COUNT] = {0};
	size_t used_count = 0;
	size_t rejected_count = 0;

	for (size_t i = 0; i < candidate_count; i++) {
		const GnssModuleState *m = candidates[i];
		float dy_m = ((float)(m->lat_e7 - med_lat_e7) * 1e-7f) * 111320.0f;
		float dx_m = ((float)(m->lon_e7 - med_lon_e7) * 1e-7f) * meters_per_deg_lon;
		float residual_m = sqrtf(dx_m * dx_m + dy_m * dy_m);

		float hdop = clampf((float)m->hdop_centi * 0.01f, 0.5f, 50.0f);
		float threshold_m = clampf(20.0f + 15.0f * hdop, 25.0f, 150.0f);

		bool reject = residual_m > threshold_m;
		update_fault_score(m->module_index, !reject, reject, residual_m, threshold_m);

		if (reject) {
			rejected_count++;
			continue;
		}

		float w = 1.0f / (hdop * hdop);
		used[used_count] = m;
		used_weights[used_count] = w;
		used_residual_m[used_count] = residual_m;
		used_count++;
	}

	if (used_count == 0) {
		for (size_t i = 0; i < candidate_count; i++) {
			update_fault_score(candidates[i]->module_index, false, true, 0.0f, 0.0f);
		}
		r.status = GNSS_FUSION_NO_FIX;
		taskENTER_CRITICAL();
		latest = r;
		taskEXIT_CRITICAL();
		return;
	}

	float sum_w = 0.0f;
	float lat_w = 0.0f;
	float lon_w = 0.0f;
	float alt_w = 0.0f;
	uint32_t hdop_sum = 0;

	float max_residual_m = 0.0f;
	for (size_t i = 0; i < used_count; i++) {
		const GnssModuleState *m = used[i];
		float w = used_weights[i];
		sum_w += w;
		lat_w += w * (float)m->lat_e7;
		lon_w += w * (float)m->lon_e7;
		alt_w += w * (float)m->alt_cm;
		hdop_sum += m->hdop_centi;
		if (used_residual_m[i] > max_residual_m) {
			max_residual_m = used_residual_m[i];
		}
	}

	r.has_fix = true;
	r.lat_e7 = (int32_t)(lat_w / sum_w);
	r.lon_e7 = (int32_t)(lon_w / sum_w);
	r.alt_cm = (int32_t)(alt_w / sum_w);
	r.used_modules = (uint8_t)used_count;
	r.rejected_modules = (uint8_t)rejected_count;
	r.max_residual_cm = clamp_u16((uint32_t)(max_residual_m * 100.0f));
	r.avg_hdop_centi = (uint16_t)(hdop_sum / used_count);

	if (used_count >= 4 && rejected_count <= 1 && r.max_residual_cm < 3000u && r.avg_hdop_centi < 250u) {
		r.status = GNSS_FUSION_OK;
	} else if (used_count >= 2 && (rejected_count >= 2 || r.max_residual_cm > 8000u || r.avg_hdop_centi > 500u)) {
		r.status = GNSS_FUSION_INTERFERENCE;
	} else {
		r.status = GNSS_FUSION_DEGRADED;
	}

	taskENTER_CRITICAL();
	latest = r;
	taskEXIT_CRITICAL();
}

void GnssFusion_Task(void *argument) {
	(void)argument;

	while (1) {
		compute_fusion();
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

bool GnssFusion_GetResult(GnssFusionResult *out) {
	if (out == NULL) {
		return false;
	}
	taskENTER_CRITICAL();
	*out = latest;
	taskEXIT_CRITICAL();
	return true;
}

bool GnssFusion_GetResultFromISR(GnssFusionResult *out) {
	if (out == NULL) {
		return false;
	}
	UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
	*out = latest;
	taskEXIT_CRITICAL_FROM_ISR(saved);
	return true;
}

bool GnssFusion_GetModuleFaultScore(uint8_t module_index, uint16_t *out_score) {
	if (out_score == NULL || module_index < 1 || module_index > GNSS_MODULE_COUNT) {
		return false;
	}
	taskENTER_CRITICAL();
	*out_score = fault_score[module_index - 1];
	taskEXIT_CRITICAL();
	return true;
}
