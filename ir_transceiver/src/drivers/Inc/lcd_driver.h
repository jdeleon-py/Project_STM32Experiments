/*
 * lcd_driver.h
 *
 *  Created on: Nov 25, 2022
 *      Author: jamesdeleon
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include <stdio.h>
#include "stm32fxx.h"

#define LCD_PD0_MASK     (uint8_t)(1 << 0)
#define LCD_PD1_MASK     (uint8_t)(1 << 1)
#define LCD_PD2_MASK     (uint8_t)(1 << 2)
#define LCD_PD3_MASK     (uint8_t)(1 << 3)
#define LCD_PD4_MASK     (uint8_t)(1 << 4)
#define LCD_PD5_MASK     (uint8_t)(1 << 5)
#define LCD_PD6_MASK     (uint8_t)(1 << 6)
#define LCD_PD7_MASK     (uint8_t)(1 << 7)

void LCD_Init(void);
void LCD_PORTS_Init(void);
void LCD_Command(uint8_t command);
void LCD_Data(unsigned char data);
void LCD_Enable(void);
void LCD_Delay(uint32_t num);

#endif /* INC_LCD_DRIVER_H_ */
