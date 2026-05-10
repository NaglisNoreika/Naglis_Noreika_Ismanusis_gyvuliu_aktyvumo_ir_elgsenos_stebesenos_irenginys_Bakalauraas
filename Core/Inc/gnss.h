/*
 * gnss.h
 *
 *  Created on: 2026-04-29
 *      Author: Naglis Noreika 2026
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include "main.h"
#include <stdint.h>

void GNSS_Init(UART_HandleTypeDef *huart);
void GNSS_Process(void);

uint8_t GNSS_IsAlive(void);
uint8_t GNSS_HasFix(void);
int32_t GNSS_GetLatitudeE6(void);
int32_t GNSS_GetLongitudeE6(void);

#endif /* INC_GNSS_H_ */
