/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 30, 2022
 *      Author: jamesdeleon
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32fxx.h"

// @GPIO_PIN_NUMBERS
// GPIO Pin Numbers
#define GPIO_PIN_NUM_0                 0
#define GPIO_PIN_NUM_1                 1
#define GPIO_PIN_NUM_2                 2
#define GPIO_PIN_NUM_3                 3
#define GPIO_PIN_NUM_4                 4
#define GPIO_PIN_NUM_5                 5
#define GPIO_PIN_NUM_6                 6
#define GPIO_PIN_NUM_7                 7
#define GPIO_PIN_NUM_8                 8
#define GPIO_PIN_NUM_9                 9
#define GPIO_PIN_NUM_10                10
#define GPIO_PIN_NUM_11                11
#define GPIO_PIN_NUM_12                12
#define GPIO_PIN_NUM_13                13
#define GPIO_PIN_NUM_14                14
#define GPIO_PIN_NUM_15                15

// @GPIO_PIN_MODES
// GPIO Pin Mode States
#define GPIO_MODE_IN                   0
#define GPIO_MODE_OUT                  1
#define GPIO_MODE_ALT_FN               2
#define GPIO_MODE_ANALOG               3
#define GPIO_MODE_IT_FT                4
#define GPIO_MODE_IT_RT                5
#define GPIO_MODE_IT_RFT               6

// @GPIO_PIN_OUTTYPES
// GPIO Pin Output States
#define GPIO_OUTTYPE_PP                0
#define GPIO_OUTTYPE_OD                1

// @GPIO_PIN_SPEEDS
// GPIO Pin Speed States
#define GPIO_SPEED_LOW                 0
#define GPIO_SPEED_MEDIUM              1
#define GPIO_SPEED_HIGH                2
#define GPIO_SPEED_VERYHIGH            3

// @GPIO_PIN_PUPD
// GPIO Pin PUPD States
#define GPIO_NO_PUPD                   0
#define GPIO_PIN_PU                    1
#define GPIO_PIN_PD                    2

// Configuration Structure Definition for a GPIO Pin
typedef struct
{
	uint8_t GPIO_PinNumber;            // Possible values @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;              // Possible values @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;             // Possible values @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPUPDControl;       // Possible values @GPIO_PIN_PUPD
	uint8_t GPIO_PinOType;             // Possible values @GPIO_PIN_OUTTYPES
	uint8_t GPIO_PinAltFuncMode;
} GPIO_PinConfig_t;

// Handle Structure for a GPIO Pin
typedef struct
{
	GPIO_RegDef_t *pGPIOx;             // This holds the base address of the GPIO port where the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;   // This holds the structure configuring the GPIO pin
} GPIO_Handle_t;

/* ************************************************ */
/* ******** API's supported by this driver ******** */
/* ************************************************ */

// Init Functions
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Peripheral Clock Control
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state);

// Data IO
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t data);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t data);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum);

// Peripheral IRQ Control
void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t state);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNum);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
