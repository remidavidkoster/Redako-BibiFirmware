################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/eth/fs/fsdata.c 

OBJS += \
./Core/Inc/eth/fs/fsdata.o 

C_DEPS += \
./Core/Inc/eth/fs/fsdata.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/eth/fs/%.o Core/Inc/eth/fs/%.su Core/Inc/eth/fs/%.cyclo: ../Core/Inc/eth/fs/%.c Core/Inc/eth/fs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-eth-2f-fs

clean-Core-2f-Inc-2f-eth-2f-fs:
	-$(RM) ./Core/Inc/eth/fs/fsdata.cyclo ./Core/Inc/eth/fs/fsdata.d ./Core/Inc/eth/fs/fsdata.o ./Core/Inc/eth/fs/fsdata.su

.PHONY: clean-Core-2f-Inc-2f-eth-2f-fs

