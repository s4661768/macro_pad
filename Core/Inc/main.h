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
#define GENERIC_REPORT_SIZE 9

/**
 *  This Struct is holds the variables that are to be reported to the computer over
 *  USB.
 *
 * Use the following functions to manipulate an instance of this struct:
 * 		void clear_generic_report(GenericReport* genRep);
 * 		void clear_generic_msg(GenericReport* genRep);
 * 		void build_generic_msg(GenericReport* genRep);
 * 		void set_report(EventBits_t bits, GenericReport* genRep, uint8_t layer);
 *
 * */
typedef struct GenericReport {
	/*
	 * A USB report follows this format where each element is a byte:
	 * 		[ reportId | Modifier | Reserved | Key1 | Key2 | Key3 | Key4 | Key5 | Key6 ]
	 *
	 * A Consumer Controller (Media) report follows this format:
	 * 		[ reportId | flags ]
	 *
	 * 	The flags byte is a group of 7 flags using the least significant bits of the byte.
	 * 		Bit 0: Next Track
	 * 		Bit 1: Previous Track
	 * 		Bit 2: Stop
	 * 		Bit 3: Play / Pause
	 * 		Bit 4: Mute
	 * 		Bit 5: Volume Up
	 * 		Bit 6: Volume Down
	 *
	 * A USB report must be followed by a report that clears the previous key. This is done
	 * so that the computer 'releases' the keystroke to prevent spamming.
	 * */
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
