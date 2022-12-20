/*
 * keypad_driver.h
 *
 *  Created on: Dec 4, 2022
 *      Author: jamesdeleon
 */

#ifndef INC_KEYPAD_DRIVER_H_
#define INC_KEYPAD_DRIVER_H_

#include "stm32fxx.h"

void Keypad_Init(void);
void Keypad_PORTS_Init(void);
uint8_t Keypad_Scan(void);
void Keypad_Delay(uint32_t num);

#endif /* INC_KEYPAD_DRIVER_H_ */
