################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/test/unit/core/test_def.c \
../Core/lwip/test/unit/core/test_mem.c \
../Core/lwip/test/unit/core/test_netif.c \
../Core/lwip/test/unit/core/test_pbuf.c \
../Core/lwip/test/unit/core/test_timers.c 

OBJS += \
./Core/lwip/test/unit/core/test_def.o \
./Core/lwip/test/unit/core/test_mem.o \
./Core/lwip/test/unit/core/test_netif.o \
./Core/lwip/test/unit/core/test_pbuf.o \
./Core/lwip/test/unit/core/test_timers.o 

C_DEPS += \
./Core/lwip/test/unit/core/test_def.d \
./Core/lwip/test/unit/core/test_mem.d \
./Core/lwip/test/unit/core/test_netif.d \
./Core/lwip/test/unit/core/test_pbuf.d \
./Core/lwip/test/unit/core/test_timers.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/test/unit/core/%.o Core/lwip/test/unit/core/%.su Core/lwip/test/unit/core/%.cyclo: ../Core/lwip/test/unit/core/%.c Core/lwip/test/unit/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-test-2f-unit-2f-core

clean-Core-2f-lwip-2f-test-2f-unit-2f-core:
	-$(RM) ./Core/lwip/test/unit/core/test_def.cyclo ./Core/lwip/test/unit/core/test_def.d ./Core/lwip/test/unit/core/test_def.o ./Core/lwip/test/unit/core/test_def.su ./Core/lwip/test/unit/core/test_mem.cyclo ./Core/lwip/test/unit/core/test_mem.d ./Core/lwip/test/unit/core/test_mem.o ./Core/lwip/test/unit/core/test_mem.su ./Core/lwip/test/unit/core/test_netif.cyclo ./Core/lwip/test/unit/core/test_netif.d ./Core/lwip/test/unit/core/test_netif.o ./Core/lwip/test/unit/core/test_netif.su ./Core/lwip/test/unit/core/test_pbuf.cyclo ./Core/lwip/test/unit/core/test_pbuf.d ./Core/lwip/test/unit/core/test_pbuf.o ./Core/lwip/test/unit/core/test_pbuf.su ./Core/lwip/test/unit/core/test_timers.cyclo ./Core/lwip/test/unit/core/test_timers.d ./Core/lwip/test/unit/core/test_timers.o ./Core/lwip/test/unit/core/test_timers.su

.PHONY: clean-Core-2f-lwip-2f-test-2f-unit-2f-core

