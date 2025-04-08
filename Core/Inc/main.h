/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define KEYBOARD_REPORT_ID 0x01
#define MEDIA_REPORT_ID 0x03
#define REPORT_SIZE 11
typedef struct KeystrokeReport {

	uint8_t mod;
	uint8_t res;
	uint8_t k1;
	uint8_t k2;
	uint8_t k3;
	uint8_t k4;
	uint8_t k5;
	uint8_t k6;
	uint8_t c0;

	uint8_t msg[REPORT_SIZE]; // Outgoing message to USB interface task

} KeystrokeReport;



#define GENERIC_REPORT_SIZE 9

typedef struct GenericReport {

	uint8_t reportId;   // 0x01 | 0x03
	uint8_t r0;  		// Mod  | report
	uint8_t r1;  		// Res  | -
	uint8_t r2;  		// k1   | -
	uint8_t r3;  		// k2   | -
	uint8_t r4;  		// k3   | -
	uint8_t r5;  		// k4   | -
	uint8_t r6;  		// k5   | -
	uint8_t r7;  		// k6   | -

	uint8_t msg[GENERIC_REPORT_SIZE]; // Outgoing message to USB interface task
} GenericReport;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C0_Pin GPIO_PIN_0
#define C0_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_1
#define C1_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_2
#define C2_GPIO_Port GPIOA
#define R0_Pin GPIO_PIN_3
#define R0_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_4
#define R1_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_5
#define R2_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_6
#define R3_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
