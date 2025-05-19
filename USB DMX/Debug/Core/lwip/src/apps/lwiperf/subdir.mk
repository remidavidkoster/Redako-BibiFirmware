################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/apps/lwiperf/lwiperf.c 

OBJS += \
./Core/lwip/src/apps/lwiperf/lwiperf.o 

C_DEPS += \
./Core/lwip/src/apps/lwiperf/lwiperf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/apps/lwiperf/%.o Core/lwip/src/apps/lwiperf/%.su Core/lwip/src/apps/lwiperf/%.cyclo: ../Core/lwip/src/apps/lwiperf/%.c Core/lwip/src/apps/lwiperf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-apps-2f-lwiperf

clean-Core-2f-lwip-2f-src-2f-apps-2f-lwiperf:
	-$(RM) ./Core/lwip/src/apps/lwiperf/lwiperf.cyclo ./Core/lwip/src/apps/lwiperf/lwiperf.d ./Core/lwip/src/apps/lwiperf/lwiperf.o ./Core/lwip/src/apps/lwiperf/lwiperf.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-apps-2f-lwiperf

