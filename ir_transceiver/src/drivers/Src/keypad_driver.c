/*
 * keypad_driver.c
 *
 *  Created on: Dec 4, 2022
 *      Author: jamesdeleon
 */

#include "keypad_driver.h"

void Keypad_Init(void)
{
	Keypad_PORTS_Init();
}


void Keypad_PORTS_Init(void)
{
	GPIO_Handle_t gpio_padRow0;
	gpio_padRow0.pGPIOx = GPIOE;
	gpio_padRow0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_7;
	gpio_padRow0.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_padRow0.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padRow0.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padRow0.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_padRow1;
	gpio_padRow1.pGPIOx = GPIOE;
	gpio_padRow1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_8;
	gpio_padRow1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_padRow1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padRow1.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padRow1.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_padRow2;
	gpio_padRow2.pGPIOx = GPIOE;
	gpio_padRow2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
	gpio_padRow2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_padRow2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padRow2.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padRow2.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_padRow3;
	gpio_padRow3.pGPIOx = GPIOE;
	gpio_padRow3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_10;
	gpio_padRow3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_padRow3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padRow3.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padRow3.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_padCol0;
	gpio_padCol0.pGPIOx = GPIOE;
	gpio_padCol0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_11;
	gpio_padCol0.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_padCol0.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padCol0.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padCol0.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_PU;

	GPIO_Handle_t gpio_padCol1;
	gpio_padCol1.pGPIOx = GPIOE;
	gpio_padCol1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	gpio_padCol1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_padCol1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padCol1.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padCol1.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_PU;

	GPIO_Handle_t gpio_padCol2;
	gpio_padCol2.pGPIOx = GPIOE;
	gpio_padCol2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	gpio_padCol2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_padCol2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padCol2.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padCol2.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_PU;

	GPIO_Handle_t gpio_padCol3;
	gpio_padCol3.pGPIOx = GPIOE;
	gpio_padCol3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	gpio_padCol3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_padCol3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_padCol3.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_padCol3.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOE, ENABLE);

	GPIO_Init(&gpio_padRow0);
	GPIO_Init(&gpio_padRow1);
	GPIO_Init(&gpio_padRow2);
	GPIO_Init(&gpio_padRow3);
	GPIO_Init(&gpio_padCol0);
	GPIO_Init(&gpio_padCol1);
	GPIO_Init(&gpio_padCol2);
	GPIO_Init(&gpio_padCol3);
}


uint8_t Keypad_Scan(void)
{
	unsigned char key;

	// check row 0 of keypad by setting it to LOW
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_7, DISABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_8, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_9, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_10, ENABLE);
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_11) == DISABLE) {key = '1';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_12) == DISABLE) {key = '2';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_13) == DISABLE) {key = '3';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_14) == DISABLE) {key = 'A';}

	// check row 1 of keypad by setting it to LOW
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_7, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_8, DISABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_9, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_10, ENABLE);
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_11) == DISABLE) {key = '4';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_12) == DISABLE) {key = '5';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_13) == DISABLE) {key = '6';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_14) == DISABLE) {key = 'B';}

	// check row 2 of keypad by setting it to LOW
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_7, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_8, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_9, DISABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_10, ENABLE);
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_11) == DISABLE) {key = '7';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_12) == DISABLE) {key = '8';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_13) == DISABLE) {key = '9';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_14) == DISABLE) {key = 'C';}

	// check row 3 of keypad by setting it to LOW
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_7, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_8, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_9, ENABLE);
	GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NUM_10, DISABLE);
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_11) == DISABLE) {key = '*';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_12) == DISABLE) {key = '0';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_13) == DISABLE) {key = '#';}
	if(GPIO_ReadFromInputPin(GPIOE, GPIO_PIN_NUM_14) == DISABLE) {key = 'D';}

	Keypad_Delay(10);

	return key;
}


void Keypad_Delay(uint32_t num)
{
	// Delay in ms
	for(; num > 0; num--)
	{
		for(uint32_t i = 0; i < 3195; i++);
	}
}


