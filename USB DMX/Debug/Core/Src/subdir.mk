################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/config.c \
../Core/Src/dmx.c \
../Core/Src/dmx_usart.c \
../Core/Src/flash_ee.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/ncm_device.c \
../Core/Src/platform.c \
../Core/Src/profiling.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/systimer.c \
../Core/Src/usart.c \
../Core/Src/usb.c \
../Core/Src/usb_config.c 

OBJS += \
./Core/Src/config.o \
./Core/Src/dmx.o \
./Core/Src/dmx_usart.o \
./Core/Src/flash_ee.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/ncm_device.o \
./Core/Src/platform.o \
./Core/Src/profiling.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/systimer.o \
./Core/Src/usart.o \
./Core/Src/usb.o \
./Core/Src/usb_config.o 

C_DEPS += \
./Core/Src/config.d \
./Core/Src/dmx.d \
./Core/Src/dmx_usart.d \
./Core/Src/flash_ee.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/ncm_device.d \
./Core/Src/platform.d \
./Core/Src/profiling.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/systimer.d \
./Core/Src/usart.d \
./Core/Src/usb.d \
./Core/Src/usb_config.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/config.cyclo ./Core/Src/config.d ./Core/Src/config.o ./Core/Src/config.su ./Core/Src/dmx.cyclo ./Core/Src/dmx.d ./Core/Src/dmx.o ./Core/Src/dmx.su ./Core/Src/dmx_usart.cyclo ./Core/Src/dmx_usart.d ./Core/Src/dmx_usart.o ./Core/Src/dmx_usart.su ./Core/Src/flash_ee.cyclo ./Core/Src/flash_ee.d ./Core/Src/flash_ee.o ./Core/Src/flash_ee.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ncm_device.cyclo ./Core/Src/ncm_device.d ./Core/Src/ncm_device.o ./Core/Src/ncm_device.su ./Core/Src/platform.cyclo ./Core/Src/platform.d ./Core/Src/platform.o ./Core/Src/platform.su ./Core/Src/profiling.cyclo ./Core/Src/profiling.d ./Core/Src/profiling.o ./Core/Src/profiling.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/systimer.cyclo ./Core/Src/systimer.d ./Core/Src/systimer.o ./Core/Src/systimer.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usb.cyclo ./Core/Src/usb.d ./Core/Src/usb.o ./Core/Src/usb.su ./Core/Src/usb_config.cyclo ./Core/Src/usb_config.d ./Core/Src/usb_config.o ./Core/Src/usb_config.su

.PHONY: clean-Core-2f-Src

