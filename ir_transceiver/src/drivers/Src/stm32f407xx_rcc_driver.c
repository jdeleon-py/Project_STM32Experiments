/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Dec 9, 2022
 *      Author: jamesdeleon
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, sysClk, temp;
	uint8_t clkSrc, ahbp, apb1p;

	clkSrc = ((RCC -> CFGR >> 2) & 0x03);
	if(clkSrc == 0)      {sysClk = 16000000;}
	else if(clkSrc == 1) {sysClk = 8000000;}
	else if(clkSrc == 2) {sysClk = RCC_GetPLLOutClock();}

	// For AHB
	temp = ((RCC -> CFGR >> 4) & 0x0F);
	ahbp = (temp < 8) ? 1 : AHB_PreScaler[temp - 8];

	// For APB1
	temp = ((RCC -> CFGR >> 10) & 0x07);
	apb1p = (temp < 4) ? 1 : APB1_PreScaler[temp - 4];

	pclk1 = (sysClk / ahbp) / apb1p;
	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t sysClk, pclk2, temp;
	uint8_t clkSrc, ahbp, apb2p;

	clkSrc = ((RCC -> CFGR >> 2) & 0x03);
	sysClk = (clkSrc == 0) ? 16000000 : 8000000;

	temp = (RCC -> CFGR >> 4) & 0x0F;
	ahbp = (temp < 0x08) ? 1 : AHB_PreScaler[temp - 8];

	temp = (RCC -> CFGR >> 13) & 0x07;
	apb2p = (temp < 0x04) ? 1 : APB1_PreScaler[temp - 4];

	pclk2 = (sysClk / ahbp) / apb2p;
	return pclk2;
}


uint32_t RCC_GetPLLOutClock(void)
{
	return 0;
}
