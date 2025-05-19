################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/apps/netbiosns/netbiosns.c 

OBJS += \
./Core/lwip/src/apps/netbiosns/netbiosns.o 

C_DEPS += \
./Core/lwip/src/apps/netbiosns/netbiosns.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/apps/netbiosns/%.o Core/lwip/src/apps/netbiosns/%.su Core/lwip/src/apps/netbiosns/%.cyclo: ../Core/lwip/src/apps/netbiosns/%.c Core/lwip/src/apps/netbiosns/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-apps-2f-netbiosns

clean-Core-2f-lwip-2f-src-2f-apps-2f-netbiosns:
	-$(RM) ./Core/lwip/src/apps/netbiosns/netbiosns.cyclo ./Core/lwip/src/apps/netbiosns/netbiosns.d ./Core/lwip/src/apps/netbiosns/netbiosns.o ./Core/lwip/src/apps/netbiosns/netbiosns.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-apps-2f-netbiosns

