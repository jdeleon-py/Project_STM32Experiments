/*
 * stm32fxx.h
 *
 *  Created on: Oct 24, 2022
 *      Author: jamesdeleon
 */

#ifndef INC_STM32FXX_H_
#define INC_STM32FXX_H_

#include <stddef.h>
#include <stdint.h>

// Generic Macros
#define _vol                         volatile
#define _weak                        __attribute__((weak))

#define ENABLE                       1
#define DISABLE                      0
#define SET                          ENABLE
#define RESET                        DISABLE
#define GPIO_PIN_SET                 SET
#define GPIO_PIN_RESET               RESET

/********************* Processor-Specific Details *********************
 *
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0                   ((_vol uint32_t*)0xE000E100)
#define NVIC_ISER1                   ((_vol uint32_t*)0xE000E104)
#define NVIC_ISER2                   ((_vol uint32_t*)0xE000E108)
#define NVIC_ISER3                   ((_vol uint32_t*)0xE000E10C)

#define NVIC_ICER0                   ((_vol uint32_t*)0xE000E180)
#define NVIC_ICER1                   ((_vol uint32_t*)0xE000E184)
#define NVIC_ICER2                   ((_vol uint32_t*)0xE000E188)
#define NVIC_ICER3                   ((_vol uint32_t*)0xE000E18C)

#define NVIC_PR_BASEADDR             ((_vol uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED       4


// Base addresses of flash and SRAM memory
#define FLASH_BASEADDR               0x08000000U
#define SRAM1_BASEADDR               0x20000000U
#define SRAM2_BASEADDR               0x2001C000U
#define ROM                          0x1FFF0000U
#define SRAM                         SRAM1_BASEADDR


// AHBx and ApBx Bus Peripheral base addresses
#define PERIPH_BASEADDR              0x40000000U
#define APB1PERIPH_BASEADDR          PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR          0x40010000U
#define AHB1PERIPH_BASEADDR          0x40020000U
#define AHB2PERIPH_BASEADDR          0x50000000U


// Base addresses of peripherals (AHB1)
#define GPIOA_BASEADDR               (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR               (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR               (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR               (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR               (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR               (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR               (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR               (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR               (AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR               (AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR               (AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x3800)


// Base addresses of peripherals (APB1)
#define TIM2_BASEADDR                (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR                (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR                (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR                (APB1PERIPH_BASEADDR + 0x0C00)

#define USART2_BASEADDR              (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR              (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR               (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR               (APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR               (APB1PERIPH_BASEADDR + 0x7800)

#define UART8_BASEADDR               (APB1PERIPH_BASEADDR + 0x7C00)
#define I2C1_BASEADDR                (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                (APB1PERIPH_BASEADDR + 0x5C00)

#define CAN1_BASEADDR                (APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR                (APB1PERIPH_BASEADDR + 0x6800)


// Base addresses of peripherals (APB2)
#define TIM1_BASEADDR                (APB2PERIPH_BASEADDR + 0x0000)

#define USART1_BASEADDR              (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR              (APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR                (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR              (APB2PERIPH_BASEADDR + 0x3800)


// RCC Register Structure Definition
typedef struct
{
	_vol uint32_t CR;                /* Clock Control Register;                                 Address Offset: 0x00      */
	_vol uint32_t PLLCFGR;           /* PLL Configuration Register;                             Address Offset: 0x04      */
	_vol uint32_t CFGR;              /* Clock Configuration Register;                           Address Offset: 0x08      */
	_vol uint32_t CIR;               /* Clock Interrupt Register;                               Address Offset: 0x0C      */
	_vol uint32_t AHB1RSTR;          /* AHB1 Peripheral Reset Register;                         Address Offset: 0x10      */
	_vol uint32_t AHB2RSTR;          /* AHB2 Peripheral Reset Register;                         Address Offset: 0x14      */
	_vol uint32_t AHB3RSTR;          /* AHB3 Peripheral Reset Register;                         Address Offset: 0x18      */
	uint32_t RESERVED0;              /* Reserved;                                               Address Offset: 0x1C      */
	_vol uint32_t APB1RSTR;          /* APB1 Peripheral Reset Register;                         Address Offset: 0x20      */
	_vol uint32_t APB2RSTR;          /* APB2 Peripheral Reset Register;                         Address Offset: 0x24      */
	uint32_t RESERVED1[2];           /* Reserved;                                               Address Offset: 0x28-0x2C */
	_vol uint32_t AHB1ENR;           /* AHB1 Peripheral Clock Enable Register;                  Address Offset: 0x30      */
	_vol uint32_t AHB2ENR;           /* AHB2 Peripheral Clock Enable Register;                  Address Offset: 0x34      */
	_vol uint32_t AHB3ENR;           /* AHB3 Peripheral Clock Enable Register;                  Address Offset: 0x38      */
	uint32_t RESERVED2;              /* Reserved;                                               Address Offset: 0x3C      */
	_vol uint32_t APB1ENR;           /* APB1 Peripheral Clock Enable Register;                  Address Offset: 0x40      */
	_vol uint32_t APB2ENR;           /* APB2 Peripheral Clock Enable Register;                  Address Offset: 0x44      */
	uint32_t RESERVED3[2];           /* Reserved;                                               Address Offset: 0x48-0x4C */
	_vol uint32_t AHB1LPENR;         /* AHB1 Peripheral Clock Enable Register (Low Power Mode); Address Offset: 0x50      */
	_vol uint32_t AHB2LPENR;         /* AHB2 Peripheral Clock Enable Register (Low Power Mode); Address Offset: 0x54      */
	_vol uint32_t AHB3LPENR;         /* AHB3 Peripheral Clock Enable Register (Low Power Mode); Address Offset: 0x58      */
	uint32_t RESERVED4;              /* Reserved;                                               Address Offset: 0x5C      */
	_vol uint32_t APB1LPENR;         /* APB1 Peripheral Clock Enable Register (Low Power Mode); Address Offset: 0x60      */
	_vol uint32_t APB2LPENR;         /* APB2 Peripheral Clock Enable Register (Low Power Mode); Address Offset: 0x64      */
	uint32_t RESERVED5[2];           /* Reserved;                                               Address Offset: 0x68-0x6C */
	_vol uint32_t BDCR;              /* Back Up Domain Register;                                Address Offset: 0x70      */
	_vol uint32_t CSR;               /* Clock Control and Status Register;                      Address Offset: 0x74      */
	uint32_t RESERVED6[2];           /* Reserved;                                               Address Offset: 0x78-0x7C */
	_vol uint32_t SSCGR;             /* Spread Spectrum Clock Generation Register;              Address Offset: 0x80      */
	_vol uint32_t PLLI2SCFGR;        /* PLLI2S Configuration Register;                          Address Offset: 0x84      */
	_vol uint32_t PLLSAICFGR;        /* PLL Configuration Register;                             Address Offset: 0x88      */
	_vol uint32_t DCKCFGR;           /* Dedicated Clock Configuration Register;                 Address Offset: 0x8C      */
} RCC_RegDef_t;


// GPIO Register Structure Definition
typedef struct
{
	_vol uint32_t MODER;             /* GPIO Port Mode Register;                               Address Offset: 0x00;      */
	_vol uint32_t OTYPER;            /* GPIO Port Output Type Register;                        Address Offset: 0x04;      */
	_vol uint32_t OSPEEDR;           /* GPIO Port Output Speed Register;                       Address Offset: 0x08;      */
	_vol uint32_t PUPDR;             /* GPIO Port PUPD Register;                               Address Offset: 0x0C;      */
	_vol uint32_t IDR;               /* GPIO Port Input Data Register;                         Address Offset: 0x10;      */
	_vol uint32_t ODR;               /* GPIO Port Output Data Register;                        Address Offset: 0x14;      */
	_vol uint32_t BSRRL;             /* GPIO Port Bit Set/Reset Low Register;                  Address Offset: 0x18;      */
	_vol uint32_t BSSRH;             /* GPIO Port Bit Set/Reset High Register;                 Address Offset: 0x1A;      */
	_vol uint32_t LCKR;              /* GPIO Port Configuration Lock Register;                 Address Offset: 0x1C;      */
	_vol uint32_t AFR[2];            /* GPIO Alternate Function Register;                      Address Offset: 0x20-0x24; */
} GPIO_RegDef_t;


// USART Register Structure Definition
typedef struct
{
	_vol uint32_t SR;                /* USART Status Register;                                 Address Offset: 0x00;      */
	_vol uint32_t DR;                /* USART Data Register;                                   Address Offset: 0x04;      */
	_vol uint32_t BRR;               /* USART Baud Rate Register;                              Address Offset: 0x08;      */
	_vol uint32_t CR1;               /* USART Control Register 1;                              Address Offset: 0x0C;      */
	_vol uint32_t CR2;               /* USART Control Register 2;                              Address Offset: 0x10;      */
	_vol uint32_t CR3;               /* USART Control Register 3;                              Address Offset: 0x14;      */
	_vol uint32_t GTPR;              /* USART Guard Time and Pre-scaling Register;             Address Offset: 0x18;      */
} USART_RegDef_t;


// EXTI Interrupt Peripheral Definition
typedef struct
{
	_vol uint32_t IMR;               /* EXTI Interrupt Mask Register;                          Address Offset: 0x00;      */
	_vol uint32_t EMR;               /* EXTI Event Mask Register;                              Address Offset: 0x04;      */
	_vol uint32_t RTSR;              /* EXTI Rising Trigger Selection Register;                Address Offset: 0x08;      */
	_vol uint32_t FTSR;              /* EXTI Falling Trigger Selection Register;               Address Offset: 0x0C;      */
	_vol uint32_t SWIER;             /* EXTI Software Interrupt Event Register;                Address Offset: 0x10;      */
	_vol uint32_t PR;                /* EXTI Pending Register;                                 Address Offset: 0x14;      */
} EXTI_RegDef_t;


// SYSCFG Peripheral Definition
typedef struct
{
	_vol uint32_t MEMRMP;            /* SYSCFG Memory Re-map Register;                         Address Offset: 0x00;      */
	_vol uint32_t PMC;               /* SYSCFG Peripheral Mode Configuration Register;         Address Offset: 0x04;      */
	_vol uint32_t EXTICR[4];         /* SYSCFG External Interrupt Configuration Registers;     Address Offset: 0x08-0x14; */
	_vol uint32_t CMPCR;             /* SYSCFG Compensation Cell Control Register;             Address Offset: 0x20;      */
} SYSCFG_RegDef_t;

// GPIO Peripheral Definitions
#define GPIOA                        ((GPIO_RegDef_t*)GPIOA_BASEADDR   )
#define GPIOB                        ((GPIO_RegDef_t*)GPIOB_BASEADDR   )
#define GPIOC                        ((GPIO_RegDef_t*)GPIOC_BASEADDR   )
#define GPIOD                        ((GPIO_RegDef_t*)GPIOD_BASEADDR   )
#define GPIOE                        ((GPIO_RegDef_t*)GPIOE_BASEADDR   )
#define GPIOF                        ((GPIO_RegDef_t*)GPIOF_BASEADDR   )
#define GPIOG                        ((GPIO_RegDef_t*)GPIOG_BASEADDR   )
#define GPIOH                        ((GPIO_RegDef_t*)GPIOH_BASEADDR   )
#define GPIOI                        ((GPIO_RegDef_t*)GPIOI_BASEADDR   )
#define GPIOJ                        ((GPIO_RegDef_t*)GPIOJ_BASEADDR   )
#define GPIOK                        ((GPIO_RegDef_t*)GPIOK_BASEADDR   )

#define USART1                       ((USART_RegDef_t*)USART1_BASEADDR )
#define USART2                       ((USART_RegDef_t*)USART2_BASEADDR )
#define USART3                       ((USART_RegDef_t*)USART3_BASEADDR )
#define UART4                        ((USART_RegDef_t*)UART4_BASEADDR  )
#define UART5                        ((USART_RegDef_t*)UART5_BASEADDR  )
#define USART6                       ((USART_RegDef_t*)USART6_BASEADDR )

#define RCC                          ((RCC_RegDef_t*)RCC_BASEADDR      )

#define EXTI                         ((EXTI_RegDef_t*)EXTI_BASEADDR    )
#define SYSCFG                       ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


// NVIC Configurations
#define IRQ_NO_EXTI0                 6
#define IRQ_NO_EXTI1                 7
#define IRQ_NO_EXTI2                 8
#define IRQ_NO_EXTI3                 9
#define IRQ_NO_EXTI5_9               23
#define IRQ_NO_EXTI10_15             40


// Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 0) )
#define GPIOB_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 1) )
#define GPIOC_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 2) )
#define GPIOD_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 3) )
#define GPIOE_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 4) )
#define GPIOF_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 5) )
#define GPIOG_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 6) )
#define GPIOH_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 7) )
#define GPIOI_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 8) )
#define GPIOJ_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 9) )
#define GPIOK_PERIPH_CLK_EN()        (RCC -> AHB1ENR |= (1 << 10))


// Clock Enable Macros for UARTx/USARTx Peripherals
#define USART1_PERIPH_CLK_EN()       (RCC -> APB2ENR |= (1 << 4) )
#define USART2_PERIPH_CLK_EN()       (RCC -> APB1ENR |= (1 << 17))
#define USART3_PERIPH_CLK_EN()       (RCC -> APB1ENR |= (1 << 18))
#define UART4_PERIPH_CLK_EN()        (RCC -> APB1ENR |= (1 << 19))
#define UART5_PERIPH_CLK_EN()        (RCC -> APB1ENR |= (1 << 20))
#define USART6_PERIPH_CLK_EN()       (RCC -> APB2ENR |= (1 << 5) )


// Clock Enable Macros for SYSCFG Peripherals
#define SYSCFG_PERIPH_CLK_EN()       (RCC -> APB2ENR |= (1 << 14))


// Clock Disable Macros for GPIOx Peripherals
#define GPIOA_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 0) )
#define GPIOB_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 1) )
#define GPIOC_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 2) )
#define GPIOD_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 3) )
#define GPIOE_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 4) )
#define GPIOF_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 5) )
#define GPIOG_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 6) )
#define GPIOH_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 7) )
#define GPIOI_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 9) )
#define GPIOK_PERIPH_CLK_DIS()       (RCC -> AHB1ENR &= ~(1 << 10))


// Clock Disable Macros for UARTx/USARTx Peripherals
#define USART1_PERIPH_CLK_DIS()      (RCC -> APB2ENR &= ~(1 << 4) )
#define USART2_PERIPH_CLK_DIS()      (RCC -> APB1ENR &= ~(1 << 17))
#define USART3_PERIPH_CLK_DIS()      (RCC -> APB1ENR &= ~(1 << 18))
#define UART4_PERIPH_CLK_DIS()       (RCC -> APB1ENR &= ~(1 << 19))
#define UART5_PERIPH_CLK_DIS()       (RCC -> APB1ENR &= ~(1 << 20))
#define USART6_PERIPH_CLK_DIS()      (RCC -> APB2ENR &= ~(1 << 5) )


// Clock Enable Macros for SYSCFG Peripherals
#define SYSCFG_PERIPH_CLK_DIS()      (RCC -> APB2ENR &= ~(1 << 14))


// Macros to reset GPIOx Peripherals
#define GPIOA_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 0);  RCC -> AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOB_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 1);  RCC -> AHB1RSTR &= ~(1 << 1); } while(0)
#define GPIOC_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 2);  RCC -> AHB1RSTR &= ~(1 << 2); } while(0)
#define GPIOD_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 3);  RCC -> AHB1RSTR &= ~(1 << 3); } while(0)
#define GPIOE_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 4);  RCC -> AHB1RSTR &= ~(1 << 4); } while(0)
#define GPIOF_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 5);  RCC -> AHB1RSTR &= ~(1 << 5); } while(0)
#define GPIOG_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 6);  RCC -> AHB1RSTR &= ~(1 << 6); } while(0)
#define GPIOH_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 7);  RCC -> AHB1RSTR &= ~(1 << 7); } while(0)
#define GPIOI_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 8);  RCC -> AHB1RSTR &= ~(1 << 8); } while(0)
#define GPIOJ_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 9);  RCC -> AHB1RSTR &= ~(1 << 9); } while(0)
#define GPIOK_PERIPH_REG_RESET()      do{RCC -> AHB1RSTR |= (1 << 10); RCC -> AHB1RSTR &= ~(1 << 10);} while(0)


// Macros to reset UARTx/USARTx Peripherals
/*
#define USART1_PERIPH_REG_RESET()     do{} while(0)
#define USART1_PERIPH_REG_RESET()     do{} while(0)
#define USART1_PERIPH_REG_RESET()     do{} while(0)
#define USART1_PERIPH_REG_RESET()     do{} while(0)
#define USART1_PERIPH_REG_RESET()     do{} while(0)
#define USART1_PERIPH_REG_RESET()     do{} while(0)
*/


// Interrupt Request Numbers for USART Peripherals
#define IRQ_NUM_USART1                37
#define IRQ_NUM_USART2                38
#define IRQ_NUM_USART3                39
#define IRQ_NUM_UART4                 52
#define IRQ_NUM_UART5                 53
#define IRQ_NUM_USART6                71


// !!! Bit Position Definitions of USART Peripheral !!!
// Bit Position Definitions of USART_CR1
#define USART_CR1_SBK                 0
#define USART_CR1_RWU                 1
#define USART_CR1_RE                  2
#define USART_CR1_TE                  3
#define USART_CR1_IDLEIE              4
#define USART_CR1_RXNEIE              5
#define USART_CR1_TCIE                6
#define USART_CR1_TXEIE               7
#define USART_CR1_PEIE                8
#define USART_CR1_PS                  9
#define USART_CR1_PCE                 10
#define USART_CR1_WAKE                11
#define USART_CR1_M                   12
#define USART_CR1_UE                  13
#define USART_CR1_OVER8               15

// Bit Position Definitions of USART_CR2
#define USART_CR2_ADD                 0
#define USART_CR2_LBDL                5
#define USART_CR2_LBDIE               6
#define USART_CR2_LBCL                8
#define USART_CR2_CPHA                9
#define USART_CR2_CPOL                10
#define USART_CR2_STOP                12
#define USART_CR2_LINEN               14

// Bit Position Definitions of USART_CR3
#define USART_CR3_EIE                 0
#define USART_CR3_IREN                1
#define USART_CR3_IRLP                2
#define USART_CR3_HDSEL               3
#define USART_CR3_NACK                4
#define USART_CR3_SCEN                5
#define USART_CR3_DMAR                6
#define USART_CR3_DMAT                7
#define USART_CR3_RTSE                8
#define USART_CR3_CTSE                9
#define USART_CR3_CTSIE               10
#define USART_CR3_ONEBIT              11

// Bit Position Definitions of USART_SR
#define USART_SR_PE                   0
#define USART_SR_FE                   1
#define USART_SR_NE                   2
#define USART_SR_ORE                  3
#define USART_SR_IDLE                 4
#define USART_SR_RXNE                 5
#define USART_SR_TC                   6
#define USART_SR_TXE                  7
#define USART_SR_LBD                  8
#define USART_SR_CTS                  9


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32FXX_H_ */
