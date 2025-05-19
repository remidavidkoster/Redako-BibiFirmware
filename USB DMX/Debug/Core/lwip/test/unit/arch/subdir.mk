################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/test/unit/arch/sys_arch.c 

OBJS += \
./Core/lwip/test/unit/arch/sys_arch.o 

C_DEPS += \
./Core/lwip/test/unit/arch/sys_arch.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/test/unit/arch/%.o Core/lwip/test/unit/arch/%.su Core/lwip/test/unit/arch/%.cyclo: ../Core/lwip/test/unit/arch/%.c Core/lwip/test/unit/arch/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-test-2f-unit-2f-arch

clean-Core-2f-lwip-2f-test-2f-unit-2f-arch:
	-$(RM) ./Core/lwip/test/unit/arch/sys_arch.cyclo ./Core/lwip/test/unit/arch/sys_arch.d ./Core/lwip/test/unit/arch/sys_arch.o ./Core/lwip/test/unit/arch/sys_arch.su

.PHONY: clean-Core-2f-lwip-2f-test-2f-unit-2f-arch

