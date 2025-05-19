################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/eth/artnet.c \
../Core/Src/eth/dhcp_server.c \
../Core/Src/eth/global.c \
../Core/Src/eth/http_custom.c \
../Core/Src/eth/ncm_netif.c 

OBJS += \
./Core/Src/eth/artnet.o \
./Core/Src/eth/dhcp_server.o \
./Core/Src/eth/global.o \
./Core/Src/eth/http_custom.o \
./Core/Src/eth/ncm_netif.o 

C_DEPS += \
./Core/Src/eth/artnet.d \
./Core/Src/eth/dhcp_server.d \
./Core/Src/eth/global.d \
./Core/Src/eth/http_custom.d \
./Core/Src/eth/ncm_netif.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/eth/%.o Core/Src/eth/%.su Core/Src/eth/%.cyclo: ../Core/Src/eth/%.c Core/Src/eth/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include/ -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-eth

clean-Core-2f-Src-2f-eth:
	-$(RM) ./Core/Src/eth/artnet.cyclo ./Core/Src/eth/artnet.d ./Core/Src/eth/artnet.o ./Core/Src/eth/artnet.su ./Core/Src/eth/dhcp_server.cyclo ./Core/Src/eth/dhcp_server.d ./Core/Src/eth/dhcp_server.o ./Core/Src/eth/dhcp_server.su ./Core/Src/eth/global.cyclo ./Core/Src/eth/global.d ./Core/Src/eth/global.o ./Core/Src/eth/global.su ./Core/Src/eth/http_custom.cyclo ./Core/Src/eth/http_custom.d ./Core/Src/eth/http_custom.o ./Core/Src/eth/http_custom.su ./Core/Src/eth/ncm_netif.cyclo ./Core/Src/eth/ncm_netif.d ./Core/Src/eth/ncm_netif.o ./Core/Src/eth/ncm_netif.su

.PHONY: clean-Core-2f-Src-2f-eth

