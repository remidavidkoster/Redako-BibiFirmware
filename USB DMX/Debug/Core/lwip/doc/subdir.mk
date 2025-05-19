################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/doc/NO_SYS_SampleCode.c \
../Core/lwip/doc/ZeroCopyRx.c 

OBJS += \
./Core/lwip/doc/NO_SYS_SampleCode.o \
./Core/lwip/doc/ZeroCopyRx.o 

C_DEPS += \
./Core/lwip/doc/NO_SYS_SampleCode.d \
./Core/lwip/doc/ZeroCopyRx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/doc/%.o Core/lwip/doc/%.su Core/lwip/doc/%.cyclo: ../Core/lwip/doc/%.c Core/lwip/doc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-doc

clean-Core-2f-lwip-2f-doc:
	-$(RM) ./Core/lwip/doc/NO_SYS_SampleCode.cyclo ./Core/lwip/doc/NO_SYS_SampleCode.d ./Core/lwip/doc/NO_SYS_SampleCode.o ./Core/lwip/doc/NO_SYS_SampleCode.su ./Core/lwip/doc/ZeroCopyRx.cyclo ./Core/lwip/doc/ZeroCopyRx.d ./Core/lwip/doc/ZeroCopyRx.o ./Core/lwip/doc/ZeroCopyRx.su

.PHONY: clean-Core-2f-lwip-2f-doc

