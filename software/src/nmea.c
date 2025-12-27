#include "nmea.h"

#include <stddef.h>
#include <string.h>

static int hex_value(char c) {
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return 10 + (c - 'a');
	}
	if (c >= 'A' && c <= 'F') {
		return 10 + (c - 'A');
	}
	return -1;
}

bool Nmea_ChecksumOk(const char *sentence) {
	if (sentence == NULL || sentence[0] != '$') {
		return false;
	}
	const char *star = strchr(sentence, '*');
	if (star == NULL || (star - sentence) < 2) {
		return false;
	}
	if (star[1] == '\0' || star[2] == '\0') {
		return false;
	}
	int hi = hex_value(star[1]);
	int lo = hex_value(star[2]);
	if (hi < 0 || lo < 0) {
		return false;
	}
	uint8_t expected = (uint8_t)((hi << 4) | lo);

	uint8_t actual = 0;
	for (const char *p = sentence + 1; p < star; p++) {
		actual ^= (uint8_t)(*p);
	}
	return actual == expected;
}

static bool parse_u32(const char *s, uint32_t *out) {
	if (s == NULL || s[0] == '\0') {
		return false;
	}
	uint32_t value = 0;
	for (const char *p = s; *p; p++) {
		if (*p < '0' || *p > '9') {
			return false;
		}
		value = (value * 10u) + (uint32_t)(*p - '0');
	}
	*out = value;
	return true;
}

static bool parse_fixed_u32(const char *s, uint32_t scale, uint32_t *out) {
	if (s == NULL || s[0] == '\0' || scale == 0) {
		return false;
	}
	uint32_t int_part = 0;
	uint32_t frac_part = 0;
	uint32_t frac_scale = 1;
	bool seen_dot = false;

	for (const char *p = s; *p; p++) {
		if (*p == '.') {
			if (seen_dot) {
				return false;
			}
			seen_dot = true;
			continue;
		}
		if (*p < '0' || *p > '9') {
			return false;
		}
		if (!seen_dot) {
			int_part = (int_part * 10u) + (uint32_t)(*p - '0');
		} else {
			if (frac_scale < scale) {
				frac_part = (frac_part * 10u) + (uint32_t)(*p - '0');
				frac_scale *= 10u;
			}
		}
	}

	while (frac_scale < scale) {
		frac_part *= 10u;
		frac_scale *= 10u;
	}

	*out = (int_part * scale) + frac_part;
	return true;
}

static bool parse_time_ms_of_day(const char *s, uint32_t *out) {
	if (s == NULL || strlen(s) < 6) {
		return false;
	}
	uint32_t hh = 0, mm = 0, ss = 0;
	if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9') {
		return false;
	}
	if (s[2] < '0' || s[2] > '9' || s[3] < '0' || s[3] > '9') {
		return false;
	}
	if (s[4] < '0' || s[4] > '9' || s[5] < '0' || s[5] > '9') {
		return false;
	}
	hh = (uint32_t)((s[0] - '0') * 10 + (s[1] - '0'));
	mm = (uint32_t)((s[2] - '0') * 10 + (s[3] - '0'));
	ss = (uint32_t)((s[4] - '0') * 10 + (s[5] - '0'));
	if (hh > 23 || mm > 59 || ss > 59) {
		return false;
	}

	uint32_t ms = 0;
	const char *dot = strchr(s, '.');
	if (dot != NULL) {
		uint32_t frac = 0;
		uint32_t frac_scale = 1;
		for (const char *p = dot + 1; *p && frac_scale < 1000; p++) {
			if (*p < '0' || *p > '9') {
				break;
			}
			frac = (frac * 10u) + (uint32_t)(*p - '0');
			frac_scale *= 10u;
		}
		while (frac_scale < 1000) {
			frac *= 10u;
			frac_scale *= 10u;
		}
		ms = frac;
	}

	*out = (((hh * 60u) + mm) * 60u + ss) * 1000u + ms;
	return true;
}

static bool parse_latlon_e7(const char *value, const char *hemi, bool is_lat, int32_t *out_e7) {
	if (value == NULL || hemi == NULL || value[0] == '\0' || hemi[0] == '\0') {
		return false;
	}
	size_t deg_digits = is_lat ? 2u : 3u;
	if (strlen(value) < deg_digits + 2u) {
		return false;
	}

	uint32_t deg = 0;
	for (size_t i = 0; i < deg_digits; i++) {
		char c = value[i];
		if (c < '0' || c > '9') {
			return false;
		}
		deg = deg * 10u + (uint32_t)(c - '0');
	}

	const char *min_str = value + deg_digits;
	if (min_str[0] < '0' || min_str[0] > '9' || min_str[1] < '0' || min_str[1] > '9') {
		return false;
	}
	uint32_t minutes_int = (uint32_t)((min_str[0] - '0') * 10 + (min_str[1] - '0'));
	uint32_t minutes_frac = 0;
	uint32_t frac_scale = 1;
	if (min_str[2] == '.') {
		for (const char *p = min_str + 3; *p && frac_scale < 1000000u; p++) {
			if (*p < '0' || *p > '9') {
				break;
			}
			minutes_frac = (minutes_frac * 10u) + (uint32_t)(*p - '0');
			frac_scale *= 10u;
		}
	}
	while (frac_scale < 1000000u) {
		minutes_frac *= 10u;
		frac_scale *= 10u;
	}

	uint32_t minutes_x1e6 = (minutes_int * 1000000u) + minutes_frac;
	int32_t signed_deg_e7 = (int32_t)(deg * 10000000u + (minutes_x1e6 / 6u));

	char h = hemi[0];
	if ((is_lat && h == 'S') || (!is_lat && h == 'W')) {
		signed_deg_e7 = -signed_deg_e7;
	} else if (!((is_lat && h == 'N') || (!is_lat && h == 'E'))) {
		return false;
	}
	*out_e7 = signed_deg_e7;
	return true;
}

static size_t tokenize_in_place(char *buf, const char **tokens, size_t max_tokens) {
	size_t count = 0;
	char *p = buf;
	while (*p && count < max_tokens) {
		tokens[count++] = p;
		while (*p && *p != ',' && *p != '*') {
			p++;
		}
		if (*p == ',' || *p == '*') {
			*p = '\0';
			p++;
		}
	}
	return count;
}

static bool has_type(const char *token0, const char *suffix3) {
	size_t n = strlen(token0);
	return n >= 3 && token0[n - 3] == suffix3[0] && token0[n - 2] == suffix3[1] &&
	       token0[n - 1] == suffix3[2];
}

bool Nmea_ParseGga(const char *sentence, NmeaGga *out) {
	if (sentence == NULL || out == NULL) {
		return false;
	}
	if (!Nmea_ChecksumOk(sentence)) {
		return false;
	}

	char tmp[96];
	size_t len = strcspn(sentence, "\r\n");
	if (len >= sizeof(tmp)) {
		return false;
	}
	memcpy(tmp, sentence, len);
	tmp[len] = '\0';
	if (tmp[0] != '$') {
		return false;
	}

	const char *tokens[20] = {0};
	size_t token_count = tokenize_in_place(tmp + 1, tokens, 20);
	if (token_count < 10) {
		return false;
	}
	if (!has_type(tokens[0], "GGA")) {
		return false;
	}

	NmeaGga parsed = {0};
	if (!parse_time_ms_of_day(tokens[1], &parsed.time_ms_of_day)) {
		return false;
	}
	if (!parse_latlon_e7(tokens[2], tokens[3], true, &parsed.lat_e7)) {
		return false;
	}
	if (!parse_latlon_e7(tokens[4], tokens[5], false, &parsed.lon_e7)) {
		return false;
	}

	uint32_t fixq = 0;
	uint32_t sats = 0;
	if (!parse_u32(tokens[6], &fixq) || !parse_u32(tokens[7], &sats)) {
		return false;
	}
	parsed.fix_quality = (uint8_t)(fixq > 255u ? 255u : fixq);
	parsed.satellites = (uint8_t)(sats > 255u ? 255u : sats);

	uint32_t hdop_x100 = 0;
	if (!parse_fixed_u32(tokens[8], 100u, &hdop_x100)) {
		return false;
	}
	parsed.hdop_centi = (uint16_t)(hdop_x100 > 65535u ? 65535u : hdop_x100);

	uint32_t alt_cm_u = 0;
	if (!parse_fixed_u32(tokens[9], 100u, &alt_cm_u)) {
		return false;
	}
	parsed.alt_cm = (int32_t)(alt_cm_u > (uint32_t)INT32_MAX ? INT32_MAX : alt_cm_u);

	*out = parsed;
	return true;
}

bool Nmea_ParseRmc(const char *sentence, NmeaRmc *out) {
	if (sentence == NULL || out == NULL) {
		return false;
	}
	if (!Nmea_ChecksumOk(sentence)) {
		return false;
	}

	char tmp[96];
	size_t len = strcspn(sentence, "\r\n");
	if (len >= sizeof(tmp)) {
		return false;
	}
	memcpy(tmp, sentence, len);
	tmp[len] = '\0';
	if (tmp[0] != '$') {
		return false;
	}

	const char *tokens[20] = {0};
	size_t token_count = tokenize_in_place(tmp + 1, tokens, 20);
	if (token_count < 10) {
		return false;
	}
	if (!has_type(tokens[0], "RMC")) {
		return false;
	}

	NmeaRmc parsed = {0};
	if (!parse_time_ms_of_day(tokens[1], &parsed.time_ms_of_day)) {
		return false;
	}
	parsed.status = (uint8_t)tokens[2][0];
	if (!parse_latlon_e7(tokens[3], tokens[4], true, &parsed.lat_e7)) {
		return false;
	}
	if (!parse_latlon_e7(tokens[5], tokens[6], false, &parsed.lon_e7)) {
		return false;
	}

	uint32_t speed_knots_x100 = 0;
	uint32_t course_deg_x100 = 0;
	(void)parse_fixed_u32(tokens[7], 100u, &speed_knots_x100);
	(void)parse_fixed_u32(tokens[8], 100u, &course_deg_x100);

	uint32_t speed_centi_ms = (uint32_t)((speed_knots_x100 * 51444u) / 100000u);
	parsed.speed_centi_ms = (uint16_t)(speed_centi_ms > 65535u ? 65535u : speed_centi_ms);
	parsed.course_centi_deg = (uint16_t)(course_deg_x100 > 65535u ? 65535u : course_deg_x100);

	*out = parsed;
	return true;
}
