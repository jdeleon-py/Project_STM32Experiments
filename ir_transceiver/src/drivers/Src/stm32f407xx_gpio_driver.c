/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 30, 2022
 *      Author: jamesdeleon
 */

#include "stm32f407xx_gpio_driver.h"

// Init Functions
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// Configure the mode of the GPIO pin
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> MODER |= temp;
	}
	else
	{
		// TO DO LATER (Interrupt Mode)
	}
	temp = 0;

	// Configure the pin speed
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;
	temp = 0;

	// Configure the PUPD settings
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPUPDControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;
	temp = 0;

	// Configure the output type
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinOType << (1 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 << (1 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;
	temp = 0;

	// Configure the alternate function registers
	if(pGPIOHandle ->  GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));
	}
	temp = 0;
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	// RCC AHB1 Peripheral Reset Register
	// resets all GPIO pin configurations to their default state
	if(pGPIOx == GPIOA)      {GPIOA_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOB) {GPIOB_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOC) {GPIOC_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOD) {GPIOD_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOE) {GPIOE_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOF) {GPIOF_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOG) {GPIOG_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOH) {GPIOH_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOI) {GPIOI_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOJ) {GPIOJ_PERIPH_REG_RESET();}
	else if(pGPIOx == GPIOK) {GPIOK_PERIPH_REG_RESET();}
}


// Peripheral Clock Control
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state)
{
	if(state == ENABLE)
	{
		if(pGPIOx == GPIOA)      {GPIOA_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOB) {GPIOB_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOC) {GPIOC_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOD) {GPIOD_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOE) {GPIOE_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOF) {GPIOF_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOG) {GPIOG_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOH) {GPIOH_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOI) {GPIOI_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOJ) {GPIOJ_PERIPH_CLK_EN();}
		else if(pGPIOx == GPIOK) {GPIOK_PERIPH_CLK_EN();}
	}
	else
	{
		if(pGPIOx == GPIOA)      {GPIOA_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOB) {GPIOB_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOC) {GPIOC_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOD) {GPIOD_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOE) {GPIOE_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOF) {GPIOF_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOG) {GPIOG_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOH) {GPIOH_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOI) {GPIOI_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOJ) {GPIOJ_PERIPH_CLK_DIS();}
		else if(pGPIOx == GPIOK) {GPIOK_PERIPH_CLK_DIS();}
	}
}


// Data IO
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx -> IDR >> pinNum) & 0x00000001);
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx -> IDR);
	return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t data)
{
	if(data == GPIO_PIN_SET) {pGPIOx -> ODR |= (1 << pinNum);}
	else {pGPIOx -> ODR &= ~(1 << pinNum);}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t data)
{
	pGPIOx -> ODR = data;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum)
{
	pGPIOx -> ODR ^= (1 << pinNum);
}


// Peripheral IRQ (Interrupt) Control
void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t state)
{
	if(state == ENABLE)
	{
		if(IRQNum <= 31)                    {*NVIC_ISER0 |= (1 << IRQNum);}        // Program ISER0 Register
		else if(IRQNum > 31 && IRQNum < 64) {*NVIC_ISER1 |= (1 << (IRQNum % 32));} // Program ISER1 Register
		else if(IRQNum >= 64 && IRQNum < 96){*NVIC_ISER2 |= (1 << (IRQNum % 64));} // Program ISER2 Register
	}
	else
	{
		if(IRQNum <= 31)                    {*NVIC_ICER0 |= (1 << IRQNum);}        // Program ICER0 Register
		else if(IRQNum > 31 && IRQNum < 64) {*NVIC_ICER1 |= (1 << (IRQNum % 32));} // Program ICER1 Register
		else if(IRQNum >= 64 && IRQNum < 96){*NVIC_ICER2 |= (1 << (IRQNum % 64));} // Program ICER2 Register
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
	// Find the IPR register
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t pinNum)
{
	if(EXTI -> PR & (1 << pinNum)) {EXTI -> PR |= (1 << pinNum);} // Clear the register
}
