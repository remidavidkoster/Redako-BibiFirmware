################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/test/unit/tcp/tcp_helper.c \
../Core/lwip/test/unit/tcp/test_tcp.c \
../Core/lwip/test/unit/tcp/test_tcp_oos.c 

OBJS += \
./Core/lwip/test/unit/tcp/tcp_helper.o \
./Core/lwip/test/unit/tcp/test_tcp.o \
./Core/lwip/test/unit/tcp/test_tcp_oos.o 

C_DEPS += \
./Core/lwip/test/unit/tcp/tcp_helper.d \
./Core/lwip/test/unit/tcp/test_tcp.d \
./Core/lwip/test/unit/tcp/test_tcp_oos.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/test/unit/tcp/%.o Core/lwip/test/unit/tcp/%.su Core/lwip/test/unit/tcp/%.cyclo: ../Core/lwip/test/unit/tcp/%.c Core/lwip/test/unit/tcp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-test-2f-unit-2f-tcp

clean-Core-2f-lwip-2f-test-2f-unit-2f-tcp:
	-$(RM) ./Core/lwip/test/unit/tcp/tcp_helper.cyclo ./Core/lwip/test/unit/tcp/tcp_helper.d ./Core/lwip/test/unit/tcp/tcp_helper.o ./Core/lwip/test/unit/tcp/tcp_helper.su ./Core/lwip/test/unit/tcp/test_tcp.cyclo ./Core/lwip/test/unit/tcp/test_tcp.d ./Core/lwip/test/unit/tcp/test_tcp.o ./Core/lwip/test/unit/tcp/test_tcp.su ./Core/lwip/test/unit/tcp/test_tcp_oos.cyclo ./Core/lwip/test/unit/tcp/test_tcp_oos.d ./Core/lwip/test/unit/tcp/test_tcp_oos.o ./Core/lwip/test/unit/tcp/test_tcp_oos.su

.PHONY: clean-Core-2f-lwip-2f-test-2f-unit-2f-tcp

