/*
 * lcd_driver.c
 *
 *  Created on: Nov 25, 2022
 *      Author: jamesdeleon
 */

#include "lcd_driver.h"

void LCD_Init(void)
{
	// Enable ports (@PORTS_Init())
	LCD_PORTS_Init();

	// Initialization sequence and modes
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_1, DISABLE);

	// LCD init commands (0x--)
	LCD_Command(0x30); // Initialization
	LCD_Command(0x38); // Mode: 8-bit data
	LCD_Command(0x01); // Clear Display
	LCD_Command(0x02); // Cursor is at Home Position
	LCD_Command(0x0F); // Display on, cursor blinking
}


void LCD_PORTS_Init(void)
{
	// Enable RCC clocks for GPIO registers
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	// Set GPIO configuration registers
	GPIO_Handle_t GPIO_RS;
	GPIO_RS.pGPIOx = GPIOC;
	GPIO_RS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	GPIO_RS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_RS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_RS.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_RS.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_RW;
	GPIO_RW.pGPIOx = GPIOC;
	GPIO_RW.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_1;
	GPIO_RW.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_RW.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_RW.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_RW.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_E;
	GPIO_E.pGPIOx = GPIOC;
	GPIO_E.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_2;
	GPIO_E.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_E.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_E.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_E.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D0;
	GPIO_D0.pGPIOx = GPIOD;
	GPIO_D0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	GPIO_D0.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D0.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D0.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D0.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D1;
	GPIO_D1.pGPIOx = GPIOD;
	GPIO_D1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_1;
	GPIO_D1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D1.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D1.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D2;
	GPIO_D2.pGPIOx = GPIOD;
	GPIO_D2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_2;
	GPIO_D2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D2.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D2.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D3;
	GPIO_D3.pGPIOx = GPIOD;
	GPIO_D3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_3;
	GPIO_D3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D3.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D3.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D4;
	GPIO_D4.pGPIOx = GPIOD;
	GPIO_D4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_4;
	GPIO_D4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D4.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D4.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D5;
	GPIO_D5.pGPIOx = GPIOD;
	GPIO_D5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GPIO_D5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D5.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D5.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D6;
	GPIO_D6.pGPIOx = GPIOD;
	GPIO_D6.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GPIO_D6.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D6.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D6.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D6.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t GPIO_D7;
	GPIO_D7.pGPIOx = GPIOD;
	GPIO_D7.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_7;
	GPIO_D7.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_D7.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GPIO_D7.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	GPIO_D7.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_RS);
	GPIO_Init(&GPIO_RW);
	GPIO_Init(&GPIO_E);
	GPIO_Init(&GPIO_D0);
	GPIO_Init(&GPIO_D1);
	GPIO_Init(&GPIO_D2);
	GPIO_Init(&GPIO_D3);
	GPIO_Init(&GPIO_D4);
	GPIO_Init(&GPIO_D5);
	GPIO_Init(&GPIO_D6);
	GPIO_Init(&GPIO_D7);
}


void LCD_Command(uint8_t command)
{
	// Set RS to LOW
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_0, DISABLE);

	// Put command on GPIO pins (D0 - D7)
	if(((uint8_t)command & LCD_PD7_MASK) == LCD_PD7_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_7, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_7, DISABLE);}

	if(((uint8_t)command & LCD_PD6_MASK) == LCD_PD6_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_6, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_6, DISABLE);}

	if(((uint8_t)command & LCD_PD5_MASK) == LCD_PD5_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_5, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_5, DISABLE);}

	if(((uint8_t)command & LCD_PD4_MASK) == LCD_PD4_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_4, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_4, DISABLE);}

	if(((uint8_t)command & LCD_PD3_MASK) == LCD_PD3_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_3, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_3, DISABLE);}

	if(((uint8_t)command & LCD_PD2_MASK) == LCD_PD2_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_2, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_2, DISABLE);}

	if(((uint8_t)command & LCD_PD1_MASK) == LCD_PD1_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_1, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_1, DISABLE);}

	if(((uint8_t)command & LCD_PD0_MASK) == LCD_PD0_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_0, ENABLE);}
	else                                                  {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_0, DISABLE);}
	LCD_Enable();
}


void LCD_Data(unsigned char data)
{
	// Set RS to HIGH
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_0, ENABLE);

	// Put data on GPIO pins (D0 - D7)
	if(((uint8_t)data & LCD_PD7_MASK) == LCD_PD7_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_7, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_7, DISABLE);}

	if(((uint8_t)data & LCD_PD6_MASK) == LCD_PD6_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_6, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_6, DISABLE);}

	if(((uint8_t)data & LCD_PD5_MASK) == LCD_PD5_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_5, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_5, DISABLE);}

	if(((uint8_t)data & LCD_PD4_MASK) == LCD_PD4_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_4, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_4, DISABLE);}

	if(((uint8_t)data & LCD_PD3_MASK) == LCD_PD3_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_3, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_3, DISABLE);}

	if(((uint8_t)data & LCD_PD2_MASK) == LCD_PD2_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_2, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_2, DISABLE);}

	if(((uint8_t)data & LCD_PD1_MASK) == LCD_PD1_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_1, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_1, DISABLE);}

	if(((uint8_t)data & LCD_PD0_MASK) == LCD_PD0_MASK) {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_0, ENABLE);}
	else                                               {GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_0, DISABLE);}
	LCD_Enable();
	LCD_Command(0x06); // Shift cursor to the right for the next character
}


void LCD_Enable(void)
{
	// Set 'enable' pin to HIGH
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_2, ENABLE);
	LCD_Delay(10); // Delay 10 ms

	// Set 'enable' pin to LOW
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_2, DISABLE);
	LCD_Delay(20); // Delay 20 ms
}


void LCD_Delay(uint32_t num)
{
	// Delay in ms
	for(; num > 0; num--)
	{
		for(uint32_t i = 0; i < 3195; i++);
	}
}
