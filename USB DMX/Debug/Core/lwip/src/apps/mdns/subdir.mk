################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/apps/mdns/mdns.c 

OBJS += \
./Core/lwip/src/apps/mdns/mdns.o 

C_DEPS += \
./Core/lwip/src/apps/mdns/mdns.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/apps/mdns/%.o Core/lwip/src/apps/mdns/%.su Core/lwip/src/apps/mdns/%.cyclo: ../Core/lwip/src/apps/mdns/%.c Core/lwip/src/apps/mdns/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include/ -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-apps-2f-mdns

clean-Core-2f-lwip-2f-src-2f-apps-2f-mdns:
	-$(RM) ./Core/lwip/src/apps/mdns/mdns.cyclo ./Core/lwip/src/apps/mdns/mdns.d ./Core/lwip/src/apps/mdns/mdns.o ./Core/lwip/src/apps/mdns/mdns.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-apps-2f-mdns

