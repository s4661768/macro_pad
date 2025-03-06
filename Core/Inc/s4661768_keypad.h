/**
 ******************************************************************************
 * @file    mylib/s4661768_keypad.h
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





#ifndef S4661768_KEYPAD_H
#define S4661768_KEYPAD_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#define INIT_STATE 0
#define RSCAN0_STATE 1
#define RSCAN1_STATE 2
#define RSCAN2_STATE 3
#define RSCAN3_STATE 4

#define NO_COLUMN 0xFF
#define COLUMN_0 0xFE
#define COLUMN_1 0xFD
#define COLUMN_2 0xFB
//#define COLUMN_3 0xF7

#define ROW_0 0x01
#define ROW_1 0x02
#define ROW_2 0x04
#define ROW_3 0x08

#define KEY0 (1 << 0)
#define KEY1 (1 << 1)
#define KEY2 (1 << 2)
//#define KEY3 (1 << 3)
#define KEY4 (1 << 4)
#define KEY5 (1 << 5)
#define KEY6 (1 << 6)
//#define KEY7 (1 << 7)
#define KEY8 (1 << 8)
#define KEY9 (1 << 9)
#define KEY10 (1 << 10)
//#define KEY11 (1 << 11)
#define KEY12 (1 << 12)
#define KEY13 (1 << 13)
#define KEY14 (1 << 14)
//#define KEY15 (1 << 15)
//#define ALL_KEYS (0xFFFF)
#define KEYBOARD_KEYS ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 12) | (1 << 13))

#define KEYPAD_COL0() keypad_writecol(COLUMN_0)
#define KEYPAD_COL1() keypad_writecol(COLUMN_1)
#define KEYPAD_COL2() keypad_writecol(COLUMN_2)
//#define KEYPAD_COL3() keypad_writecol(COLUMN_3)

#ifndef KEYPAD_VAR
#define KEYPAD_VAR
//extern EventGroupHandle_t Keypad;
#endif

// Macros for the command element of the KeyValue struct
//#define UP 0
//#define DOWN 1
//#define VACUUM_TOGGLE 2
//#define ROTATE 3
//#define NEW 4
//#define DELETE 5
//#define SYSTEM 6

/* Priority and stack definitions for Keypad task. */
#define KEYPADTASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define KEYPADTASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 6)

typedef struct KeyValue {

    int x;
    int y;
    uint8_t cmd; // To be set to one of the macros above

} KeyValue;


/** 
 * Makes the Keypad task.
*/
void s4661768_tsk_keypad_init();


/**
 * Initialises GPIO required for the PMOD keypad.
 * 
 * The columns are mapped to pins B10, B11, E14, and E15. The rows are mapped to
 * E7, E8, 10, and E12.
*/
void s4661768_reg_keypad_init();


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
void s4661768_reg_keypad_fsmprocessing();


/**
 * Getter function that returns KeypadStatus flag.
 * 
 * @return
 *  KeypadStatus <int>: flag showing whether a key press was registered or not. 1 for
 *  registered key press 0 otherwise.
*/
int s4661768_reg_keypad_read_status();


/**
 * Returns the value of a key that has been pressed.
 * 
 * Uses the row and column information, and the Keys array to find the value 
 * corresponding to a given key press.
 * 
 * @return
 *  KeypadValue <unsigned char>: the corresponding value of a key press as per
 *  'Keys' 2D array defined in s4661768_keypad.h.
*/
unsigned char s4661768_reg_keypad_read_key();

#endif
