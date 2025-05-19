################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/test/fuzz/fuzz.c 

OBJS += \
./Core/lwip/test/fuzz/fuzz.o 

C_DEPS += \
./Core/lwip/test/fuzz/fuzz.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/test/fuzz/%.o Core/lwip/test/fuzz/%.su Core/lwip/test/fuzz/%.cyclo: ../Core/lwip/test/fuzz/%.c Core/lwip/test/fuzz/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-test-2f-fuzz

clean-Core-2f-lwip-2f-test-2f-fuzz:
	-$(RM) ./Core/lwip/test/fuzz/fuzz.cyclo ./Core/lwip/test/fuzz/fuzz.d ./Core/lwip/test/fuzz/fuzz.o ./Core/lwip/test/fuzz/fuzz.su

.PHONY: clean-Core-2f-lwip-2f-test-2f-fuzz

