#include "gnss.h"
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *gnss_uart = NULL;

static uint8_t gnss_alive = 0;
static uint8_t gnss_has_fix = 0;
static uint32_t gnss_last_seen = 0;

static int32_t gnss_lat_i = 0;
static int32_t gnss_lon_i = 0;

static char nmea_line[128];
static uint16_t nmea_idx = 0;

static int32_t GNSS_NmeaToDegE6(const char *s) {
	double raw = atof(s);
	int deg = (int) (raw / 100);
	double min = raw - (deg * 100);
	double dec = deg + (min / 60.0);

	return (int32_t) (dec * 1000000.0);
}

static void GNSS_ParseGGA(const char *line) {
	uint8_t comma = 0;
	const char *p = line;

	char lat_str[16] = { 0 };
	char lon_str[16] = { 0 };
	char ns = 0;
	char ew = 0;

	uint8_t fix_quality = 0;

	while (*p) {
		if (*p == ',') {
			comma++;
			p++;

			if (comma == 2)
				strncpy(lat_str, p, sizeof(lat_str) - 1);
			else if (comma == 3)
				ns = *p;
			else if (comma == 4)
				strncpy(lon_str, p, sizeof(lon_str) - 1);
			else if (comma == 5)
				ew = *p;
			else if (comma == 6)
				fix_quality = (uint8_t) atoi(p);
			else if (comma == 7)

				break;

		} else {
			p++;
		}
	}

	gnss_has_fix = (fix_quality > 0);

	if (gnss_has_fix && lat_str[0] && lon_str[0]) {
		gnss_lat_i = GNSS_NmeaToDegE6(lat_str);
		gnss_lon_i = GNSS_NmeaToDegE6(lon_str);

		if (ns == 'S')
			gnss_lat_i = -gnss_lat_i;

		if (ew == 'W')
			gnss_lon_i = -gnss_lon_i;
	}
}

static void GNSS_ParseLine(const char *line) {
	if (line[0] != '$')
		return;

	gnss_alive = 1;
	gnss_last_seen = HAL_GetTick();

	if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
		GNSS_ParseGGA(line);
	}
}

static void GNSS_ProcessChar(uint8_t ch) {
	if (ch == '\r')
		return;

	if (ch == '\n') {
		if (nmea_idx > 0) {
			nmea_line[nmea_idx] = '\0';
			GNSS_ParseLine(nmea_line);
			nmea_idx = 0;
		}

		return;
	}

	if (nmea_idx < sizeof(nmea_line) - 1)
		nmea_line[nmea_idx++] = (char) ch;
	else
		nmea_idx = 0;
}

void GNSS_Init(UART_HandleTypeDef *huart) {
	gnss_uart = huart;

	gnss_alive = 0;
	gnss_has_fix = 0;
	gnss_last_seen = 0;

	gnss_lat_i = 0;
	gnss_lon_i = 0;

	nmea_idx = 0;
}

void GNSS_Process(void) {
	uint8_t ch;

	if (gnss_uart == NULL)
		return;

	for (uint8_t i = 0; i < 32; i++) {
		if (HAL_UART_Receive(gnss_uart, &ch, 1, 5) == HAL_OK)
			GNSS_ProcessChar(ch);
		else
			break;
	}

	if ((HAL_GetTick() - gnss_last_seen) > 3000) {
		gnss_alive = 0;
		gnss_has_fix = 0;

	}
}

uint8_t GNSS_IsAlive(void) {
	return gnss_alive;
}

uint8_t GNSS_HasFix(void) {
	return gnss_has_fix;
}

int32_t GNSS_GetLatitudeE6(void) {
	return gnss_lat_i;
}

int32_t GNSS_GetLongitudeE6(void) {
	return gnss_lon_i;
}
