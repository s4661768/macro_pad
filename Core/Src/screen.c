/*
 * screen.c
 *
 *  Created on: Jan 3, 2023
 *      Author: prism
 */

#include "screen.h"
#include "main.h"

/* Slave addr format
 *
 * [0 1 1 1 1 0     0/1     0/1_]
 *    ADDR			D/C		R/W
 *
 * D/C -> 0 for Command byte
 *
 * R/W -> 0 for Write
 */

void Initial_SSD1306(I2C_HandleTypeDef* hi2c1) {
	uint8_t ADDR = 0;
	ADDR = (0b0111100 << 1);// || (0 << 0);
//	HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut, sizeof(dataOut), 10);

//	HAL_I2C_Master_Transmit(&hi2c, ADDR, 0xAE, 1, 50);

	uint8_t data = 0;

//	Write_command(0xAE); // Display Off
	data = 0xAE;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xD5); //SET DISPLAY CLOCK
	data = 0xD5;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x80); //105HZ
	data = 0x80;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xA8); // Select Multiplex Ratio
	data = 0xA8;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x1F);
	data = 0x1F;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xD3); //Setting Display Offset
	data = 0xD3;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x00); //00H Reset, set common start
	data = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x40); //Set Display Start Line
	data = 0x40;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x8D); // Set Charge Pump
	data = 0x8D;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x14); // Enable Charge Pump
	data = 0x14; // Vcc supply: 0x14 for internal DC/DC circuit, 0x10 for external supply
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xA1); //Set Segment Re-Map Default
	data = 0xA1;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xC8); //Set COM Output Scan Direction
	data = 0xC8;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xDA); //Set COM Hardware Configuration
	data = 0xDA;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x12);
	data = 0x12;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x81); //Set Contrast Control
	data = 0x81;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xFF);
	data = 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xD9); //Set Pre-Charge period
	data = 0xD9; // Vcc supply: 0x22 for internal DC/DC circuit, 0xF1 for external supply
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x22);
	data = 0x22;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xDB); //Set Deselect Vcomh level
	data = 0xDB;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0x30);
	data = 0x30;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xA4); //Entire Display ON
	data = 0xA4;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xA6); //Set Normal Display
	data = 0xA6;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);
//	Write_command(0xAF); // Display ON
	data = 0xAF;
	HAL_I2C_Master_Transmit(hi2c1, ADDR, &data, 1, 50);

}


//// Send a byte to the command register
//void ssd1306_WriteCommand(uint8_t byte) {
//    HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
//}
//
//// Send data
//void ssd1306_WriteData(uint8_t* buffer, size_t buff_size) {
//    HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
//}


void Initial_SSD1306_2(I2C_HandleTypeDef* hi2c1) {

	uint8_t CMD = 0;
	uint8_t DATA = 0;
	CMD = (0b0111100 << 1);// || (0 << 0);
	DATA = (0b0111101 << 1);
//	HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut, sizeof(dataOut), 10);

//	HAL_I2C_Master_Transmit(&hi2c, ADDR, 0xAE, 1, 50);

	uint8_t data = 0;

//	Write_command(0xAE); // Display Off
	data = 0xAE;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0xD5); //SET DISPLAY CLOCK
	data = 0xD5;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x80); //105HZ
	data = 0x80;
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0xA8); // Select Multiplex Ratio
	data = 0xA8;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x1F);
	data = 0x1F;
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0xD3); //Setting Display Offset
	data = 0xD3;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x00); //00H Reset, set common start
	data = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0x40); //Set Display Start Line
	data = 0x40;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x8D); // Set Charge Pump
	data = 0x8D;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x14); // Enable Charge Pump
	data = 0x14; // Vcc supply: 0x14 for internal DC/DC circuit, 0x10 for external supply
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0xA1); //Set Segment Re-Map Default
	data = 0xA1;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0xC8); //Set COM Output Scan Direction
	data = 0xC8;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0xDA); //Set COM Hardware Configuration
	data = 0xDA;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x12);
	data = 0x12; // Other data sheet said 0x02
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0x81); //Set Contrast Control
	data = 0x81;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0xFF);
	data = 0xFF; //  Other data sheet said 0x8F
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0xD9); //Set Pre-Charge period
	data = 0xD9; // Vcc supply: 0x22 for internal DC/DC circuit, 0xF1 for external supply
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x22);
	data = 0x22;
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0xDB); //Set Deselect Vcomh level
	data = 0xDB;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0x30);
	data = 0x30; // Other data sheet said 0x40
	HAL_I2C_Master_Transmit(hi2c1, DATA, &data, 1, 50);
//	Write_command(0xA4); //Entire Display ON
	data = 0xA4;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0xA6); //Set Normal Display
	data = 0xA6;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);
//	Write_command(0xAF); // Display ON
	data = 0xAF;
	HAL_I2C_Master_Transmit(hi2c1, CMD, &data, 1, 50);

}
