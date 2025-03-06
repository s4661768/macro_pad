/**
  ******************************************************************************
  * @file    mylib/s4661768_oled.h
  * @author  Nathanael Mandy - 46617684
  * @date    11042023
  * @brief   SSD1306 OLED display driver. 
  ******************************************************************************
  *                           EXTERNAL FUNCTIONS
  ******************************************************************************
  * void s4661768_reg_oled_init() - Initialises the I2C communcation pins and 
  * 	initialises the SSD1306 OLED display with its library function.
  * void s4661768_tsk_oled_init() - Makes the Oled controller task. 
  */

#ifndef S4661768_OLED_H
#define S4661768_OLED_H


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


// Macros used for FSM
#define OLED_INIT_STATE 0
#define OLED_BLOCKED_STATE 1
#define OLED_UPDATE_STATE 2


// Macros used for I2C initialisation
//#define I2C_DEV_SDA_PIN		9
//#define I2C_DEV_SCL_PIN		8
//#define I2C_DEV_GPIO		GPIOB
//#define I2C_DEV_GPIO_AF 	GPIO_AF4_I2C1
//#define I2C_DEV_GPIO_CLK()	__GPIOB_CLK_ENABLE()
//
//#define I2C_DEV				I2C1
//#define I2C_DEV_CLOCKSPEED 	100000


/* Priority and stack size definitions for Oled task. */
#define OLEDTASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define OLEDTASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)

//QueueHandle_t oledQueue;

typedef struct Oled {
	
	char* string; // String to be printed
	int stringCoords[2]; // The coordinates of the string
	// NOTE: to not draw string set coords to -1,-1

	uint8_t* lines; // Array of xy coords to multiple lines.
	// Usage to draw 1 line below
	// USAGE: [xStart, yStart, xEnd, yEnd] 
	// Repeat the above structure to send multiple lines across

	uint8_t linesLength; // The length of the lines array. Set to 0 if no line is to be drawn.

	uint8_t update; // 1 if the screen is to be updated after drawing the above elements
  uint8_t clear; // 1 if the screen is to be cleared
} Oled;



void s4661768TaskOled();

/* Help function for Oled task. */
void ssd1306_draw_line(int xStart, int yStart, int xEnd, int yEnd);

/**
 * Makes Oled task.
*/
void s4661768_tsk_oled_init(void);


/** Initialises the I2C communication bus and the SSD1306 display 
 * using its library functions.
*/
void s4661768_reg_oled_init(void);

#endif
