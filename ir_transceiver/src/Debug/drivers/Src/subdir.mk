################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/keypad_driver.c \
../drivers/Src/lcd_driver.c \
../drivers/Src/stm32f407xx_gpio_driver.c \
../drivers/Src/stm32f407xx_rcc_driver.c \
../drivers/Src/stm32f407xx_usart_driver.c 

OBJS += \
./drivers/Src/keypad_driver.o \
./drivers/Src/lcd_driver.o \
./drivers/Src/stm32f407xx_gpio_driver.o \
./drivers/Src/stm32f407xx_rcc_driver.o \
./drivers/Src/stm32f407xx_usart_driver.o 

C_DEPS += \
./drivers/Src/keypad_driver.d \
./drivers/Src/lcd_driver.d \
./drivers/Src/stm32f407xx_gpio_driver.d \
./drivers/Src/stm32f407xx_rcc_driver.d \
./drivers/Src/stm32f407xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/Users/jamesdeleon/STM32CubeIDE/workspace_1.10.1/ir_transceiver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/keypad_driver.d ./drivers/Src/keypad_driver.o ./drivers/Src/keypad_driver.su ./drivers/Src/lcd_driver.d ./drivers/Src/lcd_driver.o ./drivers/Src/lcd_driver.su ./drivers/Src/stm32f407xx_gpio_driver.d ./drivers/Src/stm32f407xx_gpio_driver.o ./drivers/Src/stm32f407xx_gpio_driver.su ./drivers/Src/stm32f407xx_rcc_driver.d ./drivers/Src/stm32f407xx_rcc_driver.o ./drivers/Src/stm32f407xx_rcc_driver.su ./drivers/Src/stm32f407xx_usart_driver.d ./drivers/Src/stm32f407xx_usart_driver.o ./drivers/Src/stm32f407xx_usart_driver.su

.PHONY: clean-drivers-2f-Src

