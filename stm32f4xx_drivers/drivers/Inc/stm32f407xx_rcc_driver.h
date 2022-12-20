/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Dec 9, 2022
 *      Author: jamesdeleon
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32fxx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutClock(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
