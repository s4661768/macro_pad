/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "usbd_hid.h"
#include "s4661768_keypad.h"
#include "s4661768_oled.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_tests.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "frames.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId UsbSendTaskHandle;
/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
//extern uint8_t frames;



EventGroupHandle_t Keypad;
QueueHandle_t UsbQueue;
QueueHandle_t OledQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void UsbSendTaskInit(void const * argument);

/* USER CODE BEGIN PFP */
void s4661768TaskKeypad();
//void clear_keystroke_report(KeystrokeReport* keystrokeReport);
//void clear_msg(KeystrokeReport* keystrokeReport);
//void build_msg(KeystrokeReport* keystrokeReport);
//void set_keystroke(EventBits_t bits, KeystrokeReport* keystrokeReport, uint8_t layer);


void clear_generic_report(GenericReport* genRep);
void clear_generic_msg(GenericReport* genRep);
void build_generic_msg(GenericReport* genRep);
void set_report(EventBits_t bits, GenericReport* genRep, uint8_t layer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  /* USER CODE BEGIN 2 */
  uint8_t reportPress[8] = { 0x02, 0, 0x04, 0x05, 0x06, 0, 0, 0};
  uint8_t reportRelease[8] = { 0, 0, 0, 0, 0, 0, 0, 0};

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UsbSendTask */
  osThreadDef(UsbSendTask, UsbSendTaskInit, osPriorityNormal, 0, 128);
  UsbSendTaskHandle = osThreadCreate(osThread(UsbSendTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  s4661768_tsk_keypad_init();
  s4661768_tsk_oled_init();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//	  HIDkeyBoard.MODIFIER = 0x02; // Means 'Shift' key is being pressed
	//	  HIDkeyBoard.KEYCODE1 = 0X04; // Sends 'A'
	//	  HIDkeyBoard.KEYCODE2 = 0X05; // Sends 'B'
	//	  HIDkeyBoard.KEYCODE3 = 0X06; // Sends 'C'
	//	  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
	  USBD_HID_SendReport(&hUsbDeviceFS, reportPress, sizeof(reportPress));
	  HAL_Delay(50);

	  /* Clear everything to send released key press*/
	//	  HIDkeyBoard.MODIFIER = 0x00;
	//	  HIDkeyBoard.KEYCODE1 = 0X00;
	//	  HIDkeyBoard.KEYCODE2 = 0X00;
	//	  HIDkeyBoard.KEYCODE3 = 0X00;
	  USBD_HID_SendReport(&hUsbDeviceFS, reportRelease, sizeof(reportRelease));

	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, C0_Pin|C1_Pin|C2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : C0_Pin C1_Pin C2_Pin */
  GPIO_InitStruct.Pin = C0_Pin|C1_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/***************************************************************************************************/
/*                                      HELPER FUNCTIONS                                           */
/***************************************************************************************************/


/**
 * This function clears all attributes of the given KeystrokeReport struct.
 *
 * @param:
 * 	keystrokeReport <KeystrokeReport*>: struct used to hold details of keypress to be reported
 */
//void clear_keystroke_report(KeystrokeReport* keystrokeReport) {
//
//	clear_msg(keystrokeReport);
//
//	keystrokeReport->mod = 0;
//	keystrokeReport->res = 0;
//	keystrokeReport->k1 = 0;
//	keystrokeReport->k2 = 0;
//	keystrokeReport->k3 = 0;
//	keystrokeReport->k4 = 0;
//	keystrokeReport->k5 = 0;
//	keystrokeReport->k6 = 0;
//	keystrokeReport->c0 = 0;
//
//}

void clear_generic_report(GenericReport* genRep) {

	clear_generic_msg(genRep);

	genRep->reportId = 0;
	genRep->r0 = 0;
	genRep->r1 = 0;
	genRep->r2 = 0;
	genRep->r3 = 0;
	genRep->r4 = 0;
	genRep->r5 = 0;
	genRep->r6 = 0;
	genRep->r7 = 0;


}


/**
 * This function clears the msg attribute of the given KeystrokeReport struct.
 *
 * @param:
 * 	keystrokeReport <KeystrokeReport*>: struct used to hold details of keypress to be reported
 */
//void clear_msg(KeystrokeReport* keystrokeReport) {
//
////	keystrokeReport->msg[0] = 0;
////	keystrokeReport->msg[1] = 0;
////	keystrokeReport->msg[2] = 0;
////	keystrokeReport->msg[3] = 0;
////	keystrokeReport->msg[4] = 0;
////	keystrokeReport->msg[5] = 0;
////	keystrokeReport->msg[6] = 0;
////	keystrokeReport->msg[7] = 0;
//
//
//	/* Clear Keyboard Report */
//	keystrokeReport->msg[1] = 0;
//	keystrokeReport->msg[2] = 0;
//	keystrokeReport->msg[3] = 0;
//	keystrokeReport->msg[4] = 0;
//	keystrokeReport->msg[5] = 0;
//	keystrokeReport->msg[6] = 0;
//	keystrokeReport->msg[7] = 0;
//	keystrokeReport->msg[8] = 0;
//
//	/* Clear Media Report */
//	keystrokeReport->msg[10] = 0;
//
//}


void clear_generic_msg(GenericReport* genRep) {

	/* Clear everything but the report ID */
	genRep->msg[1] = 0;
	genRep->msg[2] = 0;
	genRep->msg[3] = 0;
	genRep->msg[4] = 0;
	genRep->msg[5] = 0;
	genRep->msg[6] = 0;
	genRep->msg[7] = 0;
	genRep->msg[8] = 0;
}

/**
 * This function populates the msg attribute of the given KeystrokeReport struct using the other
 * attributes.
 *
 * @param:
 * 	keystrokeReport <KeystrokeReport*>: struct used to hold details of keypress to be reported
 */
//void build_msg(KeystrokeReport* keystrokeReport) {
//
////	keystrokeReport->msg[0] = keystrokeReport->mod;
////	keystrokeReport->msg[1] = keystrokeReport->res;
////	keystrokeReport->msg[2] = keystrokeReport->k1;
////	keystrokeReport->msg[3] = keystrokeReport->k2;
////	keystrokeReport->msg[4] = keystrokeReport->k3;
////	keystrokeReport->msg[5] = keystrokeReport->k4;
////	keystrokeReport->msg[6] = keystrokeReport->k5;
////	keystrokeReport->msg[7] = keystrokeReport->k6;
//
//	/* Set Keyboard Report */
//	keystrokeReport->msg[0] = KEYBOARD_REPORT_ID;
// 	keystrokeReport->msg[1] = keystrokeReport->mod;
//	keystrokeReport->msg[2] = keystrokeReport->res;
//	keystrokeReport->msg[3] = keystrokeReport->k1;
//	keystrokeReport->msg[4] = keystrokeReport->k2;
//	keystrokeReport->msg[5] = keystrokeReport->k3;
//	keystrokeReport->msg[6] = keystrokeReport->k4;
//	keystrokeReport->msg[7] = keystrokeReport->k5;
//	keystrokeReport->msg[8] = keystrokeReport->k6;
//
//	/* Set Media Report */
//	keystrokeReport->msg[9] = MEDIA_REPORT_ID;
//	keystrokeReport->msg[10] = keystrokeReport->c0;
//
//}


void build_generic_msg(GenericReport* genRep) {

	/* Set Keyboard Report */
	genRep->msg[0] = genRep->reportId;
	genRep->msg[1] = genRep->r0;
	genRep->msg[2] = genRep->r1;
	genRep->msg[3] = genRep->r2;
	genRep->msg[4] = genRep->r3;
	genRep->msg[5] = genRep->r4;
	genRep->msg[6] = genRep->r5;
	genRep->msg[7] = genRep->r6;
	genRep->msg[8] = genRep->r7;

}


void set_report(EventBits_t key, GenericReport* genRep, uint8_t layer) {
//	uint8_t mute[2] = {0x03, 0x10};
//	uint8_t vol_up[2] = {0x03, 0x20};
//	uint8_t vol_down[2] = {0x03, 0x40};
//	uint8_t stop[2] = {0x03, 0x00};

	if (layer == 0) {

		switch (key) {


			case KEY0:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x04;
				break;

			case KEY1:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2= 0x05;
				break;

			case KEY2:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x06;
				break;

			case KEY4:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x07;
				break;

			case KEY5:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x08;
				break;

			case KEY6:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x09;
				break;

			case KEY8:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2= 0x0A;
				break;

			case KEY9:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x0B;
				break;

			case KEY10:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x0C;
				break;

			case KEY12:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x29;
				break;

			case KEY13:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = (0x04 | 0x01);
				genRep->r2 = 0x2B;
				break;
		}

	} else if (layer == 1) {

		switch (key) {

			case KEY0:
//				keystrokeReport->mod = 0x02;
//				keystrokeReport->k1 = 0x04;
//				keystrokeReport->k1 = 0xE2;


//				USBD_HID_SendReport(&hUsbDeviceFS, mute, sizeof(mute));
				genRep->reportId = 0x03;
				genRep->r0 = 0x10;
				break;

			case KEY1:
//				keystrokeReport->mod = 0x02;
//				keystrokeReport->k1 = 0x05;
//				keystrokeReport->k1 = 0xE9;
//				USBD_HID_SendReport(&hUsbDeviceFS, vol_up, sizeof(vol_up));
//				USBD_HID_SendReport(&hUsbDeviceFS, stop, sizeof(stop));
				genRep->reportId = 0x03;
				genRep->r0  = 0x20;
				break;

			case KEY2:
//				keystrokeReport->mod = 0x02;
//				keystrokeReport->k1 = 0x06;
//				keystrokeReport->k1 = 0xEA;
//				USBD_HID_SendReport(&hUsbDeviceFS, vol_down, sizeof(vol_down));
//				USBD_HID_SendReport(&hUsbDeviceFS, stop, sizeof(stop));
				genRep->reportId = MEDIA_REPORT_ID;
				genRep->r0 = 0x40;
				break;

			case KEY4:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = 0x02;
				genRep->r2 = 0x07;
				break;

			case KEY5:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = 0x02;
				genRep->r2 = 0x08;
				break;

			case KEY6:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = 0x02;
				genRep->r2 = 0x09;
				break;

			case KEY8:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = 0x02;
				genRep->r2 = 0x0A;
				break;

			case KEY9:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = 0x02;
				genRep->r2 = 0x0B;
				break;

			case KEY10:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0= 0x02;
				genRep->r2 = 0x0C;
				break;

			case KEY12:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r2 = 0x29;
				break;

			case KEY13:
				genRep->reportId = KEYBOARD_REPORT_ID;
				genRep->r0 = (0x04 | 0x01);
				genRep->r2= 0x2B;
				break;
		}
	}
}




void s4661768_tsk_keypad_init() {

    xTaskCreate((void *) &s4661768TaskKeypad, (const char *) "Keypad", KEYPADTASK_STACK_SIZE, NULL, KEYPADTASK_PRIORITY, NULL);
}

/**
 * Keypad task function. Calls s4661768_reg_keypad_fsmprocessing every 50 milliseconds and sets
 * an event bit if there was a keypress.
 */
void s4661768TaskKeypad() {
//    unsigned char keypress = 0xFF;
    unsigned char keypresses[4] = {0xFF, 0xFF, 0xFF};
    uint8_t keypressed = 1;
    uint8_t validpress = 0;
    Keypad = xEventGroupCreate();

    for (;;) {
    		for (int i = 0; i < 4; i++) {
    			s4661768_reg_keypad_fsmprocessing();
				vTaskDelay(20);

				if (s4661768_reg_keypad_read_status() == 0) {
					keypressed = 0;
					break;
				} else {
					keypresses[i] = s4661768_reg_keypad_read_key();
				}
				keypressed = 1;
    		}

    		for (int i = 0; i < 3; i++) {
    			if (keypressed == 0) {
    				break;
    			}

    			if (keypresses[i] != keypresses[i + 1]) {
    				keypressed = 0;
    				validpress = 0;
    				break;
    			} else {
    				keypressed = 1;
					validpress = 1;
    			}
    		}


            if (keypressed == 1 && validpress == 1) { // If there was a key press

//                keypress = s4661768_reg_keypad_read_key();

                taskENTER_CRITICAL();
                switch (keypresses[0]) {
                    case 0x00:
                        xEventGroupSetBits(Keypad, KEY0);
                        break;

                    case 0x01:
                        xEventGroupSetBits(Keypad, KEY1);
                        break;

                    case 0x02:
                        xEventGroupSetBits(Keypad, KEY2);
                        break;

                    case 0x04:
                        xEventGroupSetBits(Keypad, KEY4);
                        break;

                    case 0x05:
                        xEventGroupSetBits(Keypad, KEY5);
                        break;

                    case 0x06:
                        xEventGroupSetBits(Keypad, KEY6);
                        break;

                    case 0x08:
                        xEventGroupSetBits(Keypad, KEY8);
                        break;

                    case 0x09:
                        xEventGroupSetBits(Keypad, KEY9);
                        break;

                    case 0x0A:
                        xEventGroupSetBits(Keypad, KEY10);
                        break;

                    case 0x0C:
                        xEventGroupSetBits(Keypad, KEY12);
                        break;

                    case 0x0D:
                        xEventGroupSetBits(Keypad, KEY13);
                        break;

                    case 0x0E:
                        xEventGroupSetBits(Keypad, KEY14);
                        break;

                }

//                keypress = 0xFF; // Clearing keypress variable
                keypressed = 0;
				validpress = 0;
                taskEXIT_CRITICAL();
            }
        }
}


void s4661768TaskOled() {


	uint8_t currentState = OLED_INIT_STATE;
	uint8_t nextState = OLED_INIT_STATE;

	uint8_t frameCount = 0;
	Oled oledState; // Initialise Oled struct

	for (;;) {

		if (currentState == OLED_INIT_STATE) {
			OledQueue = xQueueCreate(4, sizeof(Oled));
			ssd1306_Init();	//Initialise SSD1306 OLED.
			nextState = OLED_BLOCKED_STATE;
		} else if (currentState == OLED_BLOCKED_STATE) {
			if (xQueueReceive(OledQueue, &oledState, 5)) { // Received from queue.
				nextState = OLED_UPDATE_STATE;
			}
			ssd1306_Fill(Black);
			ssd1306_DrawBitmap(32, 32, frames[frameCount], FRAME_WIDTH, FRAME_HEIGHT, White);
			frameCount = (frameCount + 1) % FRAME_COUNT;
			ssd1306_UpdateScreen();
		} else if (currentState == OLED_UPDATE_STATE) {
			ssd1306_Fill(Black);
			if (xQueueReceive(OledQueue, &oledState, 5)) { // Received from queue.
				nextState = OLED_UPDATE_STATE;
			}
			/* Checking if there is string to display. */
			if ((oledState.stringCoords[0] != -1) && (oledState.stringCoords[1] != -1)) {
				ssd1306_SetCursor(oledState.stringCoords[0], oledState.stringCoords[1]);
				ssd1306_WriteString(oledState.string, Font_6x8, White);
			}
			if ((oledState.linesLength != 0) && (oledState.linesLength % 4 == 0)) {

				/* Drawing lines. */
				int i = 0;
				while (i < oledState.linesLength) {
					ssd1306_draw_line(oledState.lines[i], oledState.lines[i + 1], oledState.lines[i + 2]
							   , oledState.lines[i + 3]);
					i += 4;
				}
				i = 0;

			}

			ssd1306_DrawBitmap(32, 32, frames[frameCount], FRAME_WIDTH, FRAME_HEIGHT, White);
//			ssd1306_DrawBitmap(32, 16, frames[frameCount], FRAME_WIDTH, FRAME_HEIGHT, White);
			frameCount = (frameCount + 1) % FRAME_COUNT;
			ssd1306_UpdateScreen();
		}

		currentState = nextState;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	uint8_t layer = 0;
	EventBits_t key;
//	KeystrokeReport keystrokeReport;
	GenericReport genRep;
	Oled oledStruct;
//	clear_keystroke_report(&keystrokeReport);
	clear_generic_report(&genRep);
	char oledMsg[21];
	uint8_t lines[12] = {0, 11, 127, 11,
					 	 84, 0, 84, 10};
	uint8_t welcome = 1;

	/* Infinite loop */
  	for(;;) {
		/* Making sure event group exists before attempting to use it */
		if (Keypad == NULL) { continue; }

		/* Boot up screen */
		if ((welcome == 1) && (OledQueue != NULL)) {

			  oledStruct.string = "Hello User.";
			  oledStruct.stringCoords[0] = 15;
			  oledStruct.stringCoords[1] = 10;
			  xQueueSend(OledQueue, &oledStruct, 5);

			  oledStruct.string = "Your MacroPad Is";
			  oledStruct.stringCoords[0] = 15;
			  oledStruct.stringCoords[1] = 20;
			  xQueueSend(OledQueue, &oledStruct, 5);

			  oledStruct.string = "Ready.";
			  oledStruct.stringCoords[0] = 15;
			  oledStruct.stringCoords[1] = 30;
			  xQueueSend(OledQueue, &oledStruct, 5);

			  oledStruct.update = 1;
			  xQueueSend(OledQueue, &oledStruct, 5);


			  welcome++;

		}

		vTaskDelay(60);

		key = 0;
		key = xEventGroupWaitBits(Keypad, KEYBOARD_KEYS | KEY14, pdTRUE, pdFALSE, 10);

		if (key == 0) {

		  continue;

		} else if (key == KEY14) {

		  layer = ((layer + 1) % 2);

		  /* Sending new layer information to OLED task */
		  if (OledQueue != NULL) {
			  oledStruct.clear = 1;
			  xQueueSend(OledQueue, &oledStruct, 1);

			  memset(oledMsg, '\0', sizeof(oledMsg));
			  sprintf(oledMsg, "L%d: Media     v0.0.1", layer);
			  oledStruct.string = oledMsg;
			  oledStruct.stringCoords[0] = 5;
			  oledStruct.stringCoords[1] = 0;

			  oledStruct.lines = lines;
			  oledStruct.linesLength = 8;

			  oledStruct.update = 1;
			  xQueueSend(OledQueue, &oledStruct, 1);

		  }
		  continue;
		}

		set_report(key, &genRep, layer);



		if (UsbQueue == NULL) {

//		 clear_keystroke_report(&keystrokeReport);

			clear_generic_report(&genRep);
			continue;
		}
		/* Send keystroke */
//		build_msg(&keystrokeReport);
		build_generic_msg(&genRep);
		xQueueSend(UsbQueue, &genRep.msg, 5);

		/* clear keystroke and send no key press */
//		clear_keystroke_report(&keystrokeReport);I
		clear_generic_msg(&genRep);
//		build_msg(&keystrokeReport);
//		build_generic_msg(&genRep);
		xQueueSend(UsbQueue, &genRep.msg, 5);
		osDelay(10); // Wait and send again just in case packet was missed.
		xQueueSend(UsbQueue, &genRep.msg, 5);
		clear_generic_report(&genRep);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UsbSendTaskInit */
/**
* @brief Function implementing the UsbSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsbSendTaskInit */
void UsbSendTaskInit(void const * argument)
{
  /* USER CODE BEGIN UsbSendTaskInit */

	UsbQueue = xQueueCreate(4, GENERIC_REPORT_SIZE);
	uint8_t msg[GENERIC_REPORT_SIZE] = {0};
  /* Infinite loop */
  for(;;) {

	  if (xQueueReceive(UsbQueue, &msg, 5) == pdFALSE) { continue; } // If nothing is queue do nothing

	  USBD_HID_SendReport(&hUsbDeviceFS, msg, sizeof(msg));
	  memset(msg, 0, sizeof(msg));
	  vTaskDelay(20);
  }
  /* USER CODE END UsbSendTaskInit */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
