/**
 ******************************************************************************
 * @file    mylib/s4661768_keypad.c
 * @author  Nathanael Mandy - 46617684
 * @date    20032023
 * @brief   PMOD keypad peripheral driver.
 ******************************************************************************
 *         EXTERNAL FUNCTIONS
 ******************************************************************************
 * void s4661768_reg_keypad_init() - Initialises GPIO for PMOD keypad peripheral.
 * void s4661768_reg_keypad_fsmprocessing() - FSM processing function for keypad.
 * int s4661768_reg_keypad_read_status() - KeypadStatus getter function.
 * unsigned char s4661768_reg_keypad_read_key() - Returns mathcing key value for
 *    keypress
 * void s4661768_tsk_keypad_init() - Makes Keypad control task
 */


#include "main.h"
#include "s4661768_keypad.h"

///* Priority and stack definitions for Keypad task. */
//#define KEYPADTASK_PRIORITY (tskIDLE_PRIORITY + 3)
//#define KEYPADTASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 6)

//void s4661768TaskKeypad();
void keypad_writecol(unsigned char colval);
unsigned char keypad_readrow();

int KeypadStatus = 0; // 1 when key press in registered
int KeypadFsmCurrentState = INIT_STATE;
int KeypadRowState = 0x00;


/* 2D array holding the corresponding value for each key on the
 * PMOD keypad
 */
static unsigned char Keys[4][4] = {
    {0x00, 0x01, 0x02, 0x03},
    {0x04, 0x05, 0x06, 0x07},
    {0x08, 0x09, 0x0A, 0x0B},
    {0x0C, 0x0D, 0x0E, 0x0F}};

/** 
 * Makes the Keypad task.
*/
//void s4661768_tsk_keypad_init() {
//
//    xTaskCreate((void *) &s4661768TaskKeypad, (const char *) "Keypad", KEYPADTASK_STACK_SIZE, NULL, KEYPADTASK_PRIORITY, NULL);
//}
//
///**
// * Keypad task function. Calls s4661768_reg_keypad_fsmprocessing every 50 milliseconds and sets
// * an event bit if there was a keypress.
// */
//void s4661768TaskKeypad()
//{
//    unsigned char keypress = 0xFF;
////    EventGroupHandle_t Keypad = xEventGroupCreate();
//
//    for (;;) {
//
//        s4661768_reg_keypad_fsmprocessing();
//        vTaskDelay(50);
//
//        if (KeypadStatus == 1) { // If there was a key press
//
//            keypress = s4661768_reg_keypad_read_key();
//
//            taskENTER_CRITICAL();
//            switch (keypress) {
//                case 0x00:
//                    xEventGroupSetBits(Keypad, KEY0);
//                    break;
//
//                case 0x01:
//                    xEventGroupSetBits(Keypad, KEY1);
//                    break;
//
//                case 0x02:
//                    xEventGroupSetBits(Keypad, KEY2);
//                    break;
//
//                case 0x03:
//                    xEventGroupSetBits(Keypad, KEY3);
//                    break;
//
//                case 0x04:
//                    xEventGroupSetBits(Keypad, KEY4);
//                    break;
//
//                case 0x05:
//                    xEventGroupSetBits(Keypad, KEY5);
//                    break;
//
//                case 0x06:
//                    xEventGroupSetBits(Keypad, KEY6);
//                    break;
//
//                case 0x07:
//                    xEventGroupSetBits(Keypad, KEY7);
//                    break;
//
//                case 0x08:
//                    xEventGroupSetBits(Keypad, KEY8);
//                    break;
//
////                case 0x09:
////                    xEventGroupSetBits(Keypad, KEY9);
////                    break;
////
////                case 0x0A:
////                    xEventGroupSetBits(Keypad, KEY10);
////                    break;
////
////                case 0x0B:
////                    xEventGroupSetBits(Keypad, KEY11);
////                    break;
////
////                case 0x0C:
////                    xEventGroupSetBits(Keypad, KEY12);
////                    break;
////
////                case 0x0D:
////                    xEventGroupSetBits(Keypad, KEY13);
////                    break;
////
////                case 0x0E:
////                    xEventGroupSetBits(Keypad, KEY14);
////                    break;
////
////                case 0x0F:
////                    xEventGroupSetBits(Keypad, KEY15);
////                    break;
//            }
//
//            keypress = 0xFF; // Clearing keypress variable
//            taskEXIT_CRITICAL();
//        }
//    }
//}


///**
// * Initialises GPIO required for the PMOD keypad.
// *
// * The columns are mapped to pins B10, B11, E14, and E15. The rows are mapped to
// * E7, E8, 10, and E12.
// */
//void s4661768_reg_keypad_init()
//{
//
//    /* Setting GPIOE pins 0, 2, 14, and 15 as input for the rows of the PMOD */
//
//    __GPIOE_CLK_ENABLE();
//    /* Setting as input for PMOD rows */
//    GPIOE->MODER &= ~((0x03 << (14 * 2)) | (0x03 << (15 * 2)) | (0x03 << (0 * 2)) | (0x03 << (2 * 2))); // Clearing Bits
//    // GPIOE->MODER |= ((0x01 << (14 * 2)) | (0x01 << (15 * 2))); // Setting bits
//
//    /* Setting to medium speed */
//    GPIOE->OSPEEDR &= ~((0x03 << (14 * 2)) | (0x03 << (15 * 2)) | (0x03 << (0 * 2)) | (0x03 << (2 * 2))); // Clearing Bits
//    GPIOE->OSPEEDR |= ((0x01 << (14 * 2)) | (0x01 << (15 * 2)) | (0x01 << (0 * 2)) | (0x01 << (2 * 2)));  // Setting bits
//
//    /* Setting as pull-up */
//    GPIOE->PUPDR &= ~((0x03 << (14 * 2)) | (0x03 << (15 * 2)) | (0x03 << (0 * 2)) | (0x03 << (2 * 2)));
//    GPIOE->PUPDR |= ((0x01 << (14 * 2)) | (0x01 << (15 * 2)) | (0x01 << (0 * 2)) | (0x01 << (2 * 2)));
//
//    /* Setting GPIOE pins 7, 8, 10, and 12 is output for the PMOD columns */
//    GPIOE->MODER &= ~((0x03 << (7 * 2)) | (0x03 << (8 * 2)) | (0x03 << (10 * 2)) | (0x03 << (12 * 2)));
//    GPIOE->MODER |= (0x01 << (7 * 2)) | (0x01 << (8 * 2)) | (0x01 << (10 * 2)) | (0x01 << (12 * 2));
//
//    GPIOE->OTYPER &= ~((0x03 << (7 * 2)) | (0x03 << (8 * 2)) | (0x03 << (10 * 2)) | (0x03 << (12 * 2)));
//    GPIOE->OTYPER |= (0x01 << (7 * 2)) | (0x01 << (8 * 2)) | (0x01 << (10 * 2)) | (0x01 << (12 * 2));
//
//    GPIOE->OSPEEDR &= ~((0x03 << (7 * 2)) | (0x03 << (8 * 2)) | (0x03 << (10 * 2)) | (0x03 << (12 * 2)));
//    GPIOE->OSPEEDR |= ((0x01 << (7 * 2)) | (0x01 << (8 * 2)) | (0x01 << (10 * 2)) | (0x01 << (12 * 2)));
//
//    GPIOE->PUPDR &= ~((0x03 << (7 * 2)) | (0x03 << (8 * 2)) | (0x03 << (10 * 2)) | (0x03 << (12 * 2)));
//}

/**
 * This is the finite state machine (FSM) processing function for the keypad.
 * It reads the keypad for key presses and sets flags appropriately.
 *
 * This state machine has 5 states. INIT_STATE is the 0 state, it is only in
 * this state at the beginning of program execution. In this state the
 * hardware is initialised. The other four states, (RSCANx_STATE) are states
 * corresponding to the columns of the keypad being scanned. In each of these
 * states a single column is selected and the keys are checked row by row. If a
 * key press is registered in a specific state the FSM stays in that state until
 * the key press is nolonger being registered. The FSM cycles through these four
 * states.
 */
void s4661768_reg_keypad_fsmprocessing()
{

    KeypadRowState = 0x00;

    if (KeypadFsmCurrentState == INIT_STATE)
    {

//        taskENTER_CRITICAL();
//        s4661768_reg_keypad_init();
//        taskEXIT_CRITICAL();
        KeypadFsmCurrentState = RSCAN0_STATE;
        keypad_writecol(NO_COLUMN);
    }
    else if (KeypadFsmCurrentState == RSCAN0_STATE)
    {

        KEYPAD_COL0();
        KeypadRowState = keypad_readrow();

        if (KeypadRowState != 0)
        { // Button has been pressed

            KeypadStatus = 1;

        }
        else
        { // Go to next state

            KeypadFsmCurrentState = RSCAN1_STATE;

        }
    }
    else if (KeypadFsmCurrentState == RSCAN1_STATE)
    {

        KEYPAD_COL1();
        KeypadRowState = keypad_readrow();

        if (KeypadRowState != 0)
        { // Button has been pressed

            KeypadStatus = 1;
        }
        else
        { // Go to next state

            KeypadFsmCurrentState = RSCAN2_STATE;
        }
    }
    else if (KeypadFsmCurrentState == RSCAN2_STATE)
    {

        KEYPAD_COL2();
        KeypadRowState = keypad_readrow();

        if (KeypadRowState != 0)
        { // Button has been pressed

            KeypadStatus = 1;
        }
        else
        { // Go to next state

            KeypadFsmCurrentState = RSCAN0_STATE;
        }
    }
//    else if (KeypadFsmCurrentState == RSCAN3_STATE)
//    {
//
//        KEYPAD_COL3();
//        KeypadRowState = keypad_readrow();
//
//        if (KeypadRowState != 0)
//        { // Button has been pressed
//
//            KeypadStatus = 1;
//        }
//        else
//        { // Go to next state
//
//            KeypadFsmCurrentState = RSCAN0_STATE;
//        }
//    }
}

/**
 * Selects keypad columns based on the 'colval' mask given.
 *
 * Keypad columns are active low and are selected based on the lower
 * nibble of 'colval'. A 'colval' of 0x0E will select column 0.
 *
 * @param
 *  colval <unsigned char>: mask used to select each column.
 */
void keypad_writecol(unsigned char colval)
{

    for (int i = 0; i < 4; i++) {

        if ((colval >> i) & 0x01) {

            switch (i) {

            case 3:
//                GPIOE->ODR |= (0x01 << 8);
                break;

            case 2:
            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//                GPIOE->ODR |= (0x01 << 7);
                break;

            case 1:
            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//                GPIOE->ODR |= (0x01 << 10);
                break;

            case 0:
            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
//                GPIOE->ODR |= (0x01 << 12);
                break;
            }
        } else {

            switch (i) {

            case 3:
//                GPIOE->ODR &= ~(0x01 << 8);
                break;

            case 2:
            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//                GPIOE->ODR &= ~(0x01 << 7);
                break;

            case 1:
            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//                GPIOE->ODR &= ~(0x01 << 10);
                break;

            case 0:
            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//                GPIOE->ODR &= ~(0x01 << 12);
                break;
            }
        }
    }
}

/**
 * Reads the rows of the keypad to check if a key has been pressed.
 *
 * NOTE: switches are active low
 *
 * @return
 *  rowVal <unsigned char>: bit mask showing which row is active using the lower nibble
 * of the byte. 1 for active switch, 0 else.
 */
unsigned char keypad_readrow() {
    unsigned char rowVal = 0x00;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);


	vTaskDelay(1 / portTICK_PERIOD_MS);
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
	{
		rowVal |= (0x01 << 3);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET)
	{
		rowVal |= (0x01 << 2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	}

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
	{
		rowVal |= (0x01 << 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	}

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
	{
		rowVal |= (0x01 << 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}


    return rowVal;
}

/**
 * Getter function that returns KeypadStatus flag.
 *
 * @return
 *  KeypadStatus <int>: flag showing whether a key press was registered or not. 1 for
 *  registered key press 0 otherwise.
 */
int s4661768_reg_keypad_read_status()
{

    return KeypadStatus;
}

/**
 * Returns the value of a key that has been pressed.
 *
 * Uses the row and column information, and the Keys array to find the value
 * corresponding to a given key press.
 *
 * @return
 *  KeypadValue <unsigned char>: the corresponding value of a key press as per
 *  'Keys' 2D array defined at the start of this file.
 */
unsigned char s4661768_reg_keypad_read_key()
{

    unsigned char KeypadValue = 0x00;
    uint8_t column = 0;

    if (KeypadFsmCurrentState == RSCAN0_STATE)
    {
        column = 0;
    }

    else if (KeypadFsmCurrentState == RSCAN1_STATE)
    {
        column = 1;
    }

    else if (KeypadFsmCurrentState == RSCAN2_STATE)
    {
        column = 2;
    }

    else if (KeypadFsmCurrentState == RSCAN3_STATE)
    {
        column = 3;
    }

    /* Retrieving corresponding key value */
    switch (KeypadRowState)
    {

    case ROW_0:
        KeypadValue = Keys[0][column];
        break;

    case ROW_1:
        KeypadValue = Keys[1][column];
        break;

    case ROW_2:
        KeypadValue = Keys[2][column];
        break;

    case ROW_3:
        KeypadValue = Keys[3][column];
        break;
    }

    KeypadStatus = 0;
    return KeypadValue;
}



///* Reimplemented with math */
//unsigned char s4661768_reg_keypad_read_key()
//{
//
//    unsigned char KeypadValue = 0x00;
//    uint8_t column = 0;
//    uint8_t row = 0;
//
////    if (KeypadFsmCurrentState == RSCAN0_STATE)
////    {
////        column = 0;
////    }
////
////    else if (KeypadFsmCurrentState == RSCAN1_STATE)
////    {
////        column = 1;
////    }
////
////    else if (KeypadFsmCurrentState == RSCAN2_STATE)
////    {
////        column = 2;
////    }
////
////    else if (KeypadFsmCurrentState == RSCAN3_STATE)
////    {
////        column = 3;
////    }
//
//    /* Get the column value */
//    switch(KeypadFsmCurrentState) {
//
//    case RSCAN0_STATE:
//    	column = 0;
//    	break;
//
//    case RSCAN1_STATE:
//        	column = 1;
//        	break;
//
//    case RSCAN2_STATE:
//        	column = 2;
//        	break;
//
//    case RSCAN3_STATE:
//        	column = 3;
//        	break;
//
//    }
//
//
//
//    /* Getting the row value */
//    switch (KeypadRowState)
//    {
//
//    case ROW_0:
//        	row = 0;
//        break;
//
//    case ROW_1:
//        row = 1;
//        break;
//
//    case ROW_2:
//        row = 2;
//        break;
//
//    case ROW_3:
//        row = 3;
//        break;
//    }
//
//    /* Computing the key value */
//    KeypadValue = (unsigned char) (4 * row + column);
//    KeypadStatus = 0;
//    return KeypadValue;
//}


