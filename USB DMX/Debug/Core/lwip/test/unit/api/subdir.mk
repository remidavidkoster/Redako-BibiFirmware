################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/test/unit/api/test_sockets.c 

OBJS += \
./Core/lwip/test/unit/api/test_sockets.o 

C_DEPS += \
./Core/lwip/test/unit/api/test_sockets.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/test/unit/api/%.o Core/lwip/test/unit/api/%.su Core/lwip/test/unit/api/%.cyclo: ../Core/lwip/test/unit/api/%.c Core/lwip/test/unit/api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-test-2f-unit-2f-api

clean-Core-2f-lwip-2f-test-2f-unit-2f-api:
	-$(RM) ./Core/lwip/test/unit/api/test_sockets.cyclo ./Core/lwip/test/unit/api/test_sockets.d ./Core/lwip/test/unit/api/test_sockets.o ./Core/lwip/test/unit/api/test_sockets.su

.PHONY: clean-Core-2f-lwip-2f-test-2f-unit-2f-api

