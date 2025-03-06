/**
  ******************************************************************************
  * @file    mylib/s4661768_oled.c
  * @author  Nathanael Mandy - 46617684
  * @date    11042023
  * @brief   SSD1306 OLED display driver. 
  ******************************************************************************
  *                           EXTERNAL FUNCTIONS
  ******************************************************************************
  * s4661768_reg_oled_init() - Initialises the I2C communcation pins and 
  * 	initialises the SSD1306 OLED display with its library function.
  * void s4661768_tsk_oled_init() - Makes the Oled controller task.
  */

#include <stdio.h>
#include "s4661768_oled.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "main.h"




/**
 * Makes Oled task.
*/
void s4661768_tsk_oled_init() {

	xTaskCreate((void *) &s4661768TaskOled, (const char *) "Oled", OLEDTASK_STACK_SIZE, NULL, OLEDTASK_PRIORITY, NULL);

}


/**
 * Controls the SSD1306 OLED panel. Using the variables in Oled struct it draws lines and strings
 * at the specified coordinates. Refer to Oled struct documentation for usage.
*/
//void s4661768TaskOled() {
//
//	hardware_init();
//	uint8_t currentState = OLED_INIT_STATE;
//	uint8_t nextState = OLED_INIT_STATE;
//
//
//	Oled oledState; // Initialise Oled struct
//
//	for (;;) {
//
//		if (currentState == OLED_INIT_STATE) {
//			oledQueue = xQueueCreate(4, sizeof(Oled));
////			s4661768_reg_oled_init();
//			ssd1306_Init();	//Initialise SSD1306 OLED.
//			nextState = OLED_BLOCKED_STATE;
//
//		} else if (currentState == OLED_BLOCKED_STATE) {
//			if (oledQueue == NULL) { continue; } // If queue doesn't exist continue through loop.
//
//			if (xQueueReceive(oledQueue, &oledState, 5)) { // Receive from queue.
//				nextState = OLED_UPDATE_STATE;
//			}
//
//		} else if (currentState == OLED_UPDATE_STATE) {
//			if (oledQueue == NULL) { continue; }
//
//			taskENTER_CRITICAL();
//			if (oledState.clear == 1) { // Clearing screen.
//				ssd1306_Fill(SSD1306_BLACK);
//			}
//
//			/* Checking if there is string to display. */
//			if ((oledState.stringCoords[0] != -1) && (oledState.stringCoords[1] != -1)) {
//				ssd1306_SetCursor(oledState.stringCoords[0], oledState.stringCoords[1]);
//				ssd1306_WriteString(oledState.string, Font_6x8, SSD1306_WHITE);
//			}
//
//			/* Making sure OLED.lines array is formatted correctly. */
//			if ((oledState.linesLength != 0) && (oledState.linesLength % 4 == 0)) {
//
//				/* Drawing lines. */
//				int i = 0;
//				while (i < oledState.linesLength) {
//					ssd1306_draw_line(oledState.lines[i], oledState.lines[i + 1], oledState.lines[i + 2]
//							   , oledState.lines[i + 3]);
//					i += 4;
//				}
//				i = 0;
//
//			}
//
//			if (oledState.update == 1) { // Update screen
//				ssd1306_UpdateScreen();
//			}
//			taskEXIT_CRITICAL();
//			nextState = OLED_BLOCKED_STATE;
//		}
//		currentState = nextState;
//	}
//}



/** Initialises the I2C communication bus and the SSD1306 display 
 * using its library functions.
*/
//void s4661768_reg_oled_init() {

//	uint32_t pclk1;
//	uint32_t freqrange;
//
//	// Enable GPIO clock
//	I2C_DEV_GPIO_CLK();
//
//	//******************************************************
//	// IMPORTANT NOTE: SCL Must be Initialised BEFORE SDA
//	//******************************************************
//
//	//Clear and Set Alternate Function for pin (lower ARF register)
//	MODIFY_REG(I2C_DEV_GPIO->AFR[1], ((0x0F) << ((I2C_DEV_SCL_PIN-8) * 4)) | ((0x0F) << ((I2C_DEV_SDA_PIN-8)* 4)), ((I2C_DEV_GPIO_AF << ((I2C_DEV_SCL_PIN-8) * 4)) | (I2C_DEV_GPIO_AF << ((I2C_DEV_SDA_PIN-8)) * 4)));
//
//	//Clear and Set Alternate Function Push Pull Mode
//	MODIFY_REG(I2C_DEV_GPIO->MODER, ((0x03 << (I2C_DEV_SCL_PIN * 2)) | (0x03 << (I2C_DEV_SDA_PIN * 2))), ((GPIO_MODE_AF_OD << (I2C_DEV_SCL_PIN * 2)) | (GPIO_MODE_AF_OD << (I2C_DEV_SDA_PIN * 2))));
//
//	//Set low speed.
//	SET_BIT(I2C_DEV_GPIO->OSPEEDR, (GPIO_SPEED_LOW << I2C_DEV_SCL_PIN) | (GPIO_SPEED_LOW << I2C_DEV_SDA_PIN));
//
//	//Set Bit for Push/Pull output
//	SET_BIT(I2C_DEV_GPIO->OTYPER, ((0x01 << I2C_DEV_SCL_PIN) | (0x01 << I2C_DEV_SDA_PIN)));
//
//	//Clear and set bits for no push/pull
//	MODIFY_REG(I2C_DEV_GPIO->PUPDR, (0x03 << (I2C_DEV_SCL_PIN * 2)) | (0x03 << (I2C_DEV_SDA_PIN * 2)), (GPIO_PULLUP << (I2C_DEV_SCL_PIN * 2)) | (GPIO_PULLUP << (I2C_DEV_SDA_PIN * 2)));
//
//	// Configure the I2C peripheral
//	// Enable I2C peripheral clock
//	__I2C1_CLK_ENABLE();
//
//	// Disable the selected I2C peripheral
//	CLEAR_BIT(I2C_DEV->CR1, I2C_CR1_PE);
//
//  	pclk1 = HAL_RCC_GetPCLK1Freq();			// Get PCLK1 frequency
//  	freqrange = I2C_FREQRANGE(pclk1);		// Calculate frequency range
//
//  	//I2Cx CR2 Configuration - Configure I2Cx: Frequency range
//  	MODIFY_REG(I2C_DEV->CR2, I2C_CR2_FREQ, freqrange);
//
//	// I2Cx TRISE Configuration - Configure I2Cx: Rise Time
//  	MODIFY_REG(I2C_DEV->TRISE, I2C_TRISE_TRISE, I2C_RISE_TIME(freqrange, I2C_DEV_CLOCKSPEED));
//
//   	// I2Cx CCR Configuration - Configure I2Cx: Speed
//  	MODIFY_REG(I2C_DEV->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), I2C_SPEED(pclk1, I2C_DEV_CLOCKSPEED, I2C_DUTYCYCLE_2));
//
//   	// I2Cx CR1 Configuration - Configure I2Cx: Generalcall and NoStretch mode
//  	MODIFY_REG(I2C_DEV->CR1, (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH), (I2C_GENERALCALL_DISABLE| I2C_NOSTRETCH_DISABLE));
//
//   	// I2Cx OAR1 Configuration - Configure I2Cx: Own Address1 and addressing mode
//  	MODIFY_REG(I2C_DEV->OAR1, (I2C_OAR1_ADDMODE | I2C_OAR1_ADD8_9 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0), I2C_ADDRESSINGMODE_7BIT);
//
//   	// I2Cx OAR2 Configuration - Configure I2Cx: Dual mode and Own Address2
//  	MODIFY_REG(I2C_DEV->OAR2, (I2C_OAR2_ENDUAL | I2C_OAR2_ADD2), I2C_DUALADDRESS_DISABLE);
//
//  	// Enable the selected I2C peripheral
//	SET_BIT(I2C_DEV->CR1, I2C_CR1_PE);

//}


/**
 * Uses ssd1306_DrawPixel() to draw line at specified coordinates.
 * 
 * @params:
 *  xStart, yStart, xEnd, yEnd <int>: the x,y start and end coordinates of the line to be drawn.
 * 
 * NOTE: Start coordinates must be less or equal to end coordinates andbe within the OLEDs 
 * dimensions.
*/
void ssd1306_draw_line(int xStart, int yStart, int xEnd, int yEnd) {

	if ((xStart > xEnd) || (yStart > yEnd)) { // Checking if coords are valid.
		return;
	}

	
	if (xStart == xEnd) {

		while (yStart <= yEnd) {
			ssd1306_SetCursor(xStart, yStart);

			ssd1306_DrawPixel(xStart, yStart, White);
			yStart++;
			
		}

	} else if (yStart == yEnd) {

		while (xStart <= xEnd) {
			ssd1306_SetCursor(xStart, yStart);

			ssd1306_DrawPixel(xStart, yStart, White);
			xStart++;
		}
	}
}
