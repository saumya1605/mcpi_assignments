/*
 * i2c_lcd.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Nilesh
 */

#ifndef MY_ICD_I2C_H_
#define MY_ICD_I2C_H_

#include "my_i2c.h"

#define LCD_SLA_W		0x4E
#define LCD_SLA_R		0x4F
//#define LCD_SLA_W		0x7E
//#define LCD_SLA_R		0x7F

#define LCD_CLEAR				0x01
#define LCD_SET_8BIT			0x30
#define LCD_SET_4BIT   			0x20
#define LCD_FN_SET_4BIT_2LINES	0x28
#define LCD_DISP_CTRL  			0x08
#define LCD_DISP_ON				0x0C
#define LCD_ENTRY_MODE			0x06
#define LCD_LINE1				0x80
#define LCD_LINE2				0xC0

#define LCD_RS	0
#define LCD_RW	1
#define LCD_EN	2
#define LCD_BL	3

#define LCD_CMD		0
#define LCD_DATA	1

int Lcd_Init(void);
void Lcd_Write4BitAndCtrl(uint8_t val);
void Lcd_WriteByte(uint8_t rs,uint8_t val);
void Lcd_Puts(uint8_t line, char str[]);
void Lcd_Clear(void);

#endif /* MY_ICD_I2C_H_ */
