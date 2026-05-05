/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "gnss.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	ANIMAL_STATE_UNKNOWN = 0,
	ANIMAL_STATE_CALM,
	ANIMAL_STATE_EAT,
	ANIMAL_STATE_WALK
} AnimalState_t;

typedef struct {

	uint16_t calm_seconds;
	uint16_t eat_seconds;
	uint16_t walk_seconds;
	uint16_t unknown_seconds;
} AnimalStats_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//-----------------------IMU
#define IMU_ADDR                (0x6A << 1)
#define REG_FIFO_CTRL3          0x08
#define REG_FIFO_CTRL4          0x09
#define REG_FIFO_CTRL5          0x0A
#define REG_CTRL1_XL            0x10
#define REG_CTRL3_C             0x12
#define REG_WHO_AM_I            0x0F
#define REG_FIFO_STATUS1        0x3A
#define REG_FIFO_STATUS2        0x3B
#define REG_FIFO_DATA_OUT_L     0x3E

#define FIFO_READ_PERIOD_MS     20000UL
#define FIFO_MODE_BYPASS        0x00
#define FIFO_MODE_FIFO_26HZ     0x11

//-----------------------IMU

#define Ispejimas_kai_baterijos_lygis      25
#define Ispejimo_mirksejimo_intervalas   180000UL
#define Mirksejimo_periodas        200UL
#define Mirksejimu_skaicius          6

//-----lorawan
#define Lorawan_siuntimo_intervalas    120000UL

//GNSS
#define GNSS_siuntimo_intervalas     (Lorawan_siuntimo_intervalas * 2UL)

//---------

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

AnimalStats_t g_animal_stats = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t rn_ok = 0;
uint8_t imu_ok = 0;
uint8_t gnss_ok = 0;

static void Debug_USART3_Send(const char *msg) {
	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

static void RN_SendCmd(const char *cmd) {
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef RN_ReadLine(char *out, uint16_t maxlen,
		uint32_t timeout) {
	uint16_t idx = 0;
	uint8_t ch;
	uint32_t start = HAL_GetTick();

	while ((HAL_GetTick() - start) < timeout) {
		if (HAL_UART_Receive(&huart2, &ch, 1, 10) == HAL_OK) {
			if (ch == '\r')
				continue;

			if (ch == '\n') {
				out[idx] = '\0';
				return HAL_OK;
			}

			if (idx < maxlen - 1) {
				out[idx++] = ch;
			}
		}
	}

	out[0] = '\0';
	return HAL_TIMEOUT;
}

static uint8_t RN_CheckAlive(void) {
	char line[64];

	RN_SendCmd("sys get ver");

	if (RN_ReadLine(line, sizeof(line), 2000) == HAL_OK && line[0] != '\0') {
		Debug_USART3_Send("RN versija: ");
		Debug_USART3_Send(line);
		Debug_USART3_Send("\r\n");
		return 1;
	}

	Debug_USART3_Send("Is RN atsakymo negauta\r\n");
	return 0;
}

//------------------------------IMU
static HAL_StatusTypeDef IMU_WriteReg(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 100);
}

static HAL_StatusTypeDef IMU_ReadReg(uint8_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value,
			1, 100);
}

static HAL_StatusTypeDef IMU_ReadMulti(uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf,
			len, 100);
}
//----------------------------------------------------------------

static void LSM6DSL_FIFO_Init(void) {
	uint8_t who;

	if (IMU_ReadReg(REG_WHO_AM_I, &who) != HAL_OK) {
		return;
	}

	if (who != 0x6A) {
		imu_ok = 0;
		return;
	}

	imu_ok = 1;

	/* FIFO i bypass pries konfiguruojant */
	IMU_WriteReg(REG_FIFO_CTRL5, FIFO_MODE_BYPASS);

	/* BDU + auto increment */
	IMU_WriteReg(REG_CTRL3_C, 0x44);

	/* Accel: 26 Hz, +-4g */
	IMU_WriteReg(REG_CTRL1_XL, 0x28);

	/* Tik accelerometer i FIFO */
	IMU_WriteReg(REG_FIFO_CTRL3, 0x01);

	/* FIFO_CTRL4 = 0 */
	IMU_WriteReg(REG_FIFO_CTRL4, 0x00);

	/* ODR_FIFO = 26 Hz, FIFO mode */
	IMU_WriteReg(REG_FIFO_CTRL5, FIFO_MODE_FIFO_26HZ);
}
//----------------------------------------------
//FIFO isvalymas

//----------------------------------------------------
static uint16_t LSM6DSL_FIFO_GetLevel(void) {
	uint8_t status1 = 0, status2 = 0;

	if (IMU_ReadReg(REG_FIFO_STATUS1, &status1) != HAL_OK)
		return 0;

	if (IMU_ReadReg(REG_FIFO_STATUS2, &status2) != HAL_OK)
		return 0;

	return (uint16_t) (((status2 & 0x03) << 8) | status1);
}
static void LSM6DSL_FIFO_Reset(void) {
	IMU_WriteReg(REG_FIFO_CTRL5, FIFO_MODE_BYPASS);
	HAL_Delay(2);
	IMU_WriteReg(REG_FIFO_CTRL5, FIFO_MODE_FIFO_26HZ);
	HAL_Delay(2);
}
static HAL_StatusTypeDef LSM6DSL_FIFO_ReadSample(int16_t *x, int16_t *y,
		int16_t *z) {
	uint8_t buf[6];

	if (IMU_ReadMulti(REG_FIFO_DATA_OUT_L, buf, 6) != HAL_OK)
		return HAL_ERROR;

	*x = (int16_t) ((buf[1] << 8) | buf[0]);
	*y = (int16_t) ((buf[3] << 8) | buf[2]);
	*z = (int16_t) ((buf[5] << 8) | buf[4]);

	return HAL_OK;
}

// klasifikuojamos busenos
static AnimalState_t ClassifyAnimalState(float activity_metric, float z_min,
		float z_max) {
	float z_range = z_max - z_min;

	if (activity_metric < 0.03f)
		return ANIMAL_STATE_CALM;

	if (activity_metric <= 0.18f) {
		if (z_range > 0.4f)
			return ANIMAL_STATE_CALM;  // vartymasis / gulėjimo padėties pokytis
		else
			return ANIMAL_STATE_EAT;     // nedidelis pasikartojantis judesys
	}

	return ANIMAL_STATE_WALK;
}
//----------------------------------------------------------
//
static void AnimalStats_AddWindow(AnimalState_t state, uint16_t seconds) {

	switch (state) {
	case ANIMAL_STATE_CALM:
		g_animal_stats.calm_seconds += seconds;
		break;

	case ANIMAL_STATE_EAT:
		g_animal_stats.eat_seconds += seconds;
		break;

	case ANIMAL_STATE_WALK:
		g_animal_stats.walk_seconds += seconds;
		break;

	default:
		g_animal_stats.unknown_seconds += seconds;
		break;
	}
}
//-----------------
//20 s apdorojimas

static void IMU_Process20sBlock(void) {
	uint16_t fifo_words;
	uint16_t samples;
	uint16_t i;

	int16_t x, y, z;
	float ax, ay, az;
	float mag;

	float activity_metric = 0.0f;
	float prev_mag = 0.0f;
	float sum_delta = 0.0f;

	float z_min = 100.0f;
	float z_max = -100.0f;

	AnimalState_t state;

	fifo_words = LSM6DSL_FIFO_GetLevel();
	samples = fifo_words / 3;   // 1 accel sample = 3 words

	if (samples == 0) {
		LSM6DSL_FIFO_Reset();
		return;
	}

	for (i = 0; i < samples; i++) {
		if (LSM6DSL_FIFO_ReadSample(&x, &y, &z) != HAL_OK) {
			break;
		}

		ax = (float) x * 0.122f / 1000.0f;
		ay = (float) y * 0.122f / 1000.0f;
		az = (float) z * 0.122f / 1000.0f;

		if (az < z_min)
			z_min = az;
		if (az > z_max)
			z_max = az;

		mag = ax * ax + ay * ay + az * az;

		if (i > 0) {
			sum_delta += fabsf(mag - prev_mag);
		}

		prev_mag = mag;
	}

	if (i < 2) {
		LSM6DSL_FIFO_Reset();
		return;
	}
	activity_metric = sum_delta / (float) (i - 1);

	state = ClassifyAnimalState(activity_metric, z_min, z_max);
	AnimalStats_AddWindow(state, 20);

	LSM6DSL_FIFO_Reset();
}

//------------

//Lorawan zinute
static void BytesToHexString(const uint8_t *data, uint8_t len, char *out) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		sprintf(&out[i * 2], "%02X", data[i]);
	}
	out[len * 2] = '\0';
}

static void BuildAnimalPayload(uint8_t *payload, uint8_t calm_min,
		uint8_t eat_min, uint8_t walk_min, uint8_t battery_percent,
		int32_t lat_i, int32_t lon_i) {
	payload[0] = calm_min;
	payload[1] = eat_min;
	payload[2] = walk_min;
	payload[3] = battery_percent;

	payload[4] = (uint8_t) ((lat_i >> 24) & 0xFF);
	payload[5] = (uint8_t) ((lat_i >> 16) & 0xFF);
	payload[6] = (uint8_t) ((lat_i >> 8) & 0xFF);
	payload[7] = (uint8_t) (lat_i & 0xFF);

	payload[8] = (uint8_t) ((lon_i >> 24) & 0xFF);
	payload[9] = (uint8_t) ((lon_i >> 16) & 0xFF);
	payload[10] = (uint8_t) ((lon_i >> 8) & 0xFF);
	payload[11] = (uint8_t) (lon_i & 0xFF);
}

//baterijos ikrova

static uint8_t GetBatteryPercent(void) {
	uint32_t adc_raw = 0;
	uint32_t pradzia_tick;

	float adc_itampa;
	float baterijos_ikrova;

	HAL_GPIO_WritePin(Matavimo_valdymas_GPIO_Port, Matavimo_valdymas_Pin,
			GPIO_PIN_SET);

	pradzia_tick = HAL_GetTick();
	while ((HAL_GetTick() - pradzia_tick) < 20) {
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		adc_raw = HAL_ADC_GetValue(&hadc1);
	} else {
		HAL_ADC_Stop(&hadc1);
		HAL_GPIO_WritePin(Matavimo_valdymas_GPIO_Port, Matavimo_valdymas_Pin,
				GPIO_PIN_RESET);
		return 255;
	}

	HAL_ADC_Stop(&hadc1);

	HAL_GPIO_WritePin(Matavimo_valdymas_GPIO_Port, Matavimo_valdymas_Pin,
			GPIO_PIN_RESET);

	adc_itampa = ((float) adc_raw / 4095.0f) * 3.3f;

	baterijos_ikrova = adc_itampa * ((30000.0f + 100000.0f) / 100000.0f);
	baterijos_ikrova = baterijos_ikrova * 1.024f;

	if (baterijos_ikrova >= 4.20f)
		return 100;

	if (baterijos_ikrova <= 3.30f)
		return 0;

	return (uint8_t) (((baterijos_ikrova - 3.30f) / 0.90f) * 100.0f);

}
//

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	GNSS_Init(&huart1);
	/* USER CODE BEGIN 2 */

	uint32_t last_blink = 0;
	uint8_t checked = 0;
	uint8_t blink_count = 0;
	uint8_t led_state = 0;

	//baterijos busena
	uint32_t mirksejimo_periodas = 0;
	uint32_t mirksejimo_greitis = 0;
	uint8_t kiek_kartu_sumirksejo = 0;
	uint8_t ar_dabar_mirksi = 0;

	//
	uint32_t last_lora_join_attempt = 0;
	uint8_t lora_joined = 0;
	uint32_t last_lora_tx_tick = 0;

	uint32_t last_gnss_location_update = 0;

	uint8_t payload[12];
	char payload_hex[25];
	char tx_cmd[64];

	uint8_t battery_percent = GetBatteryPercent();

	int32_t lat_i = 0;
	int32_t lon_i = 0;

	HAL_GPIO_WritePin(Indikacinis_LED_GPIO_Port, Indikacinis_LED_Pin,
			GPIO_PIN_RESET);

	uint32_t imu_last_process_tick = HAL_GetTick();
	uint32_t start_time = HAL_GetTick();
	LSM6DSL_FIFO_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint32_t now = HAL_GetTick();
		/* GNSS visada veikia */
		GNSS_Process();
		gnss_ok = GNSS_IsAlive();

		/* IMU kas 20 s */
		if ((now - imu_last_process_tick) >= FIFO_READ_PERIOD_MS) {
			imu_last_process_tick = now;
			IMU_Process20sBlock();
		}

		char line[128];

		/* Pradinis patikrinimas po 3 s */
		if (!checked && (now - start_time >= 3000)) {
			checked = 1;

			rn_ok = RN_CheckAlive();

			if (!rn_ok) {
				HAL_GPIO_WritePin(Indikacinis_LED_GPIO_Port,
				Indikacinis_LED_Pin, GPIO_PIN_SET);
			} else {
				blink_count = 0;
				last_blink = now;

			}

		}

		/* LoRaWAN konfigūracija ir JOIN - tik vieną kartą */
		if (checked && rn_ok && !lora_joined
				&& (last_lora_join_attempt == 0
						|| now - last_lora_join_attempt >= 60000UL)) {

			last_lora_join_attempt = now;

			RN_SendCmd("mac set deveui 0004A30B010B5FBC");
			RN_ReadLine(line, sizeof(line), 2000);

			RN_SendCmd("mac set appeui 0000000000000000");
			RN_ReadLine(line, sizeof(line), 2000);

			RN_SendCmd("mac set appkey 02FA9A38A1E8915F2697DA53954C79E8");
			RN_ReadLine(line, sizeof(line), 2000);

			RN_SendCmd("mac set adr on");
			RN_ReadLine(line, sizeof(line), 2000);

			RN_SendCmd("mac save");
			RN_ReadLine(line, sizeof(line), 2000);

			RN_SendCmd("mac join otaa");

			RN_ReadLine(line, sizeof(line), 3000);

			if (RN_ReadLine(line, sizeof(line), 15000) == HAL_OK) {
				if (strcmp(line, "accepted") == 0) {
					lora_joined = 1;
					last_lora_tx_tick = now - Lorawan_siuntimo_intervalas;
				}
			}
		}
		/* LoRaWAN siuntimas kas 2 minutes */
		if (lora_joined
				&& (now - last_lora_tx_tick >= Lorawan_siuntimo_intervalas)) {
			last_lora_tx_tick = now;

			uint8_t calm_min = (g_animal_stats.calm_seconds + 30) / 60;
			uint8_t eat_min = (g_animal_stats.eat_seconds + 30) / 60;
			uint8_t walk_min = (g_animal_stats.walk_seconds + 30) / 60;
			battery_percent = GetBatteryPercent();

			if ((now - last_gnss_location_update >= GNSS_siuntimo_intervalas)
					|| last_gnss_location_update == 0) {
				if (GNSS_HasFix()) {
					lat_i = GNSS_GetLatitudeE6();
					lon_i = GNSS_GetLongitudeE6();
					last_gnss_location_update = now;
				}
			}

			BuildAnimalPayload(payload, calm_min, eat_min, walk_min,
					battery_percent, lat_i, lon_i);

			BytesToHexString(payload, 12, payload_hex);

			sprintf(tx_cmd, "mac tx uncnf 1 %s", payload_hex);

			RN_SendCmd(tx_cmd);

			RN_ReadLine(line, sizeof(line), 1500);
			RN_ReadLine(line, sizeof(line), 5000);

			g_animal_stats.calm_seconds = 0;
			g_animal_stats.eat_seconds = 0;
			g_animal_stats.walk_seconds = 0;
			g_animal_stats.unknown_seconds = 0;

		}

		/* LED būsena */
		uint8_t target_blinks = 0;

		if (battery_percent != 255
				&& battery_percent <= Ispejimas_kai_baterijos_lygis
				&& ar_dabar_mirksi == 0) {
			if (now - mirksejimo_periodas >= Ispejimo_mirksejimo_intervalas) // 3 minutės
			{
				ar_dabar_mirksi = 1;
				kiek_kartu_sumirksejo = 0;
				mirksejimo_greitis = now;
				mirksejimo_periodas = now;

				led_state = 0;
				HAL_GPIO_WritePin(Indikacinis_LED_GPIO_Port,
				Indikacinis_LED_Pin, GPIO_PIN_RESET);
			}
		}

		// Jeigu dabar vyksta baterijos mirksėjimas
		if (ar_dabar_mirksi) {
			if (now - mirksejimo_greitis >= Mirksejimo_periodas) {
				mirksejimo_greitis = now;

				led_state = !led_state;

				HAL_GPIO_WritePin(
				Indikacinis_LED_GPIO_Port,
				Indikacinis_LED_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

				kiek_kartu_sumirksejo++;
			}

			// 3 mirksniai = 6 perjungimai: ON/OFF ON/OFF ON/OFF
			if (kiek_kartu_sumirksejo >= Mirksejimu_skaicius) {
				ar_dabar_mirksi = 0;
				kiek_kartu_sumirksejo = 0;

				led_state = 0;
				HAL_GPIO_WritePin(Indikacinis_LED_GPIO_Port,
				Indikacinis_LED_Pin, GPIO_PIN_RESET);
			}
		} else {
			if (rn_ok && imu_ok && gnss_ok)
				target_blinks = 12;
			else if (rn_ok && imu_ok)
				target_blinks = 10;
			else if (rn_ok && gnss_ok)
				target_blinks = 8;
			else if (imu_ok && gnss_ok)
				target_blinks = 6;
			else if (rn_ok)
				target_blinks = 4;
			else if (imu_ok)
				target_blinks = 2;
			else if (gnss_ok)
				target_blinks = 1;
			else
				target_blinks = 0;

			if (checked && target_blinks > 0 && blink_count < target_blinks) {
				if (now - last_blink >= 200) {
					last_blink = now;
					led_state = !led_state;

					HAL_GPIO_WritePin(
					Indikacinis_LED_GPIO_Port,
					Indikacinis_LED_Pin,
							led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

					blink_count++;
				}
			}

			if (checked && target_blinks > 0 && blink_count >= target_blinks) {
				led_state = 0;
				HAL_GPIO_WritePin(Indikacinis_LED_GPIO_Port,
				Indikacinis_LED_Pin, GPIO_PIN_RESET);
			}

			if (checked && target_blinks == 0) {
				led_state = 1;
				HAL_GPIO_WritePin(Indikacinis_LED_GPIO_Port,
				Indikacinis_LED_Pin, GPIO_PIN_SET);
			}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
