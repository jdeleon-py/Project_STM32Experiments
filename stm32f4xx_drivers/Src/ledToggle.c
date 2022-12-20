/*
 * ledToggle.c
 *
 *  Created on: Nov 1, 2022
 *      Author: jamesdeleon
 */

/*
 * This program utilizes the custom driver to toggle the on-board green LED (PD12)
 * and allows for the user to toggle the LED with the on-board user button.
 */

#include <stdint.h>
#include "stm32fxx.h"

#define BTN_PRESSED ENABLE
#define DELAY       400000

void delay(void)
{
	for(uint32_t i = 0; i < DELAY; i++);
}

int main(void)
{
	GPIO_Handle_t gpio_led;
	gpio_led.pGPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_led.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTTYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_button;
	gpio_button.pGPIOx = GPIOA;
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	gpio_button.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&gpio_button);
	GPIO_Init(&gpio_led);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NUM_0) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUM_12);
			delay();
		}
	}

	return 0;
}
