/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Nov 20, 2022
 *      Author: jamesdeleon
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32fxx.h"

// @USART_Mode
// Possible USART Modes
#define USART_MODE_RX              0
#define USART_MODE_TX              1
#define USART_MODE_TXRX            2

// @USART_Baud
// Possible USART Baud Rates
#define USART_STD_BAUD_1200        1200
#define USART_STD_BAUD_2400        2400
#define USART_STD_BAUD_9600        9600
#define USART_STD_BAUD_19200       19200
#define USART_STD_BAUD_38400       38400
#define USART_STD_BAUD_57600       57600
#define USART_STD_BAUD_115200      115200
#define USART_STD_BAUD_230400      230400
#define USART_STD_BAUD_460800      460800
#define USART_STD_BAUD_921600      921600
#define USART_STD_BAUD_2M          2000000
#define USART_STD_BAUD_3M          3000000

// @USART_StopBitNum
// Possible Stop Bit Numbers
#define USART_STOPBITS_1           0
#define USART_STOPBITS_0_5         1
#define USART_STOPBITS_2           2
#define USART_STOPBITS_1_5         3

// @USART_WordLength
// Possible USART Word Lengths
#define USART_WORDLENGTH_8         0
#define USART_WORDLENGTH_9         1

// @USART_ParityControl
// Possible Parity Control Configurations
#define USART_PARITY_DISABLE       0
#define USART_PARITY_EN_EVEN       1
#define USART_PARITY_EN_ODD        2

// @USART_HWFlowControl
// Possible Hardware Flow Control Configurations
#define USART_HW_FLOW_CTRL_NONE    0
#define USART_HW_FLOW_CTRL_CTS     1
#define USART_HW_FLOW_CTRL_RTS     2
#define USART_HW_FLOW_CTRL_CTS_RTS 3

// USART Flags
#define USART_FLAG_TXE             (1 << USART_SR_TXE)
#define USART_FLAG_RXNE            (1 << USART_SR_RXNE)
#define USART_FLAG_TC              (1 << USART_SR_TC)

//USART Application States
#define USART_READY                0
#define USART_BUSY_RX              1
#define USART_BUSY_TX              2

#define USART_EVENT_TX_CMPLT       0
#define USART_EVENT_RX_CMPLT       1
#define USART_EVENT_IDLE           2
#define USART_EVENT_CTS            3
#define USART_EVENT_PE             4
#define USART_ERR_FE               5
#define USART_ERR_NE               6
#define USART_ERR_ORE              7

// USART Peripheral Configuration Structure Definition
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_StopBitNum;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
} USART_Config_t;

// USART Peripheral Handling Structure Definition
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTXBuffer;
	uint8_t *pRXBuffer;
	uint32_t TXLength;
	uint32_t RXLength;
	uint8_t TXBusyState;
	uint8_t RXBusyState;
} USART_Handle_t;

/* ************************************************ */
/* ******** API's supported by this driver ******** */
/* ************************************************ */

// Initialization Functions
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

// Peripheral Clock Control
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t state);

// Data IO
void USART_SendData(USART_Handle_t *pUSARTx, uint8_t *pTXBuffer, uint32_t length);
void USART_ReceiveData(USART_Handle_t *pUSARTx, uint8_t *pRXBuffer, uint32_t length);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t length);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t length);

// IRQ Configuration and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNum, uint8_t state);
void USART_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

// Other Peripheral Control API's
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t state);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t statusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t statusFlagName);
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
