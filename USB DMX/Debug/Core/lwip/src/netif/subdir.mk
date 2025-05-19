################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/netif/bridgeif.c \
../Core/lwip/src/netif/bridgeif_fdb.c \
../Core/lwip/src/netif/ethernet.c \
../Core/lwip/src/netif/lowpan6.c \
../Core/lwip/src/netif/lowpan6_ble.c \
../Core/lwip/src/netif/lowpan6_common.c \
../Core/lwip/src/netif/slipif.c \
../Core/lwip/src/netif/zepif.c 

OBJS += \
./Core/lwip/src/netif/bridgeif.o \
./Core/lwip/src/netif/bridgeif_fdb.o \
./Core/lwip/src/netif/ethernet.o \
./Core/lwip/src/netif/lowpan6.o \
./Core/lwip/src/netif/lowpan6_ble.o \
./Core/lwip/src/netif/lowpan6_common.o \
./Core/lwip/src/netif/slipif.o \
./Core/lwip/src/netif/zepif.o 

C_DEPS += \
./Core/lwip/src/netif/bridgeif.d \
./Core/lwip/src/netif/bridgeif_fdb.d \
./Core/lwip/src/netif/ethernet.d \
./Core/lwip/src/netif/lowpan6.d \
./Core/lwip/src/netif/lowpan6_ble.d \
./Core/lwip/src/netif/lowpan6_common.d \
./Core/lwip/src/netif/slipif.d \
./Core/lwip/src/netif/zepif.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/netif/%.o Core/lwip/src/netif/%.su Core/lwip/src/netif/%.cyclo: ../Core/lwip/src/netif/%.c Core/lwip/src/netif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include/ -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-netif

clean-Core-2f-lwip-2f-src-2f-netif:
	-$(RM) ./Core/lwip/src/netif/bridgeif.cyclo ./Core/lwip/src/netif/bridgeif.d ./Core/lwip/src/netif/bridgeif.o ./Core/lwip/src/netif/bridgeif.su ./Core/lwip/src/netif/bridgeif_fdb.cyclo ./Core/lwip/src/netif/bridgeif_fdb.d ./Core/lwip/src/netif/bridgeif_fdb.o ./Core/lwip/src/netif/bridgeif_fdb.su ./Core/lwip/src/netif/ethernet.cyclo ./Core/lwip/src/netif/ethernet.d ./Core/lwip/src/netif/ethernet.o ./Core/lwip/src/netif/ethernet.su ./Core/lwip/src/netif/lowpan6.cyclo ./Core/lwip/src/netif/lowpan6.d ./Core/lwip/src/netif/lowpan6.o ./Core/lwip/src/netif/lowpan6.su ./Core/lwip/src/netif/lowpan6_ble.cyclo ./Core/lwip/src/netif/lowpan6_ble.d ./Core/lwip/src/netif/lowpan6_ble.o ./Core/lwip/src/netif/lowpan6_ble.su ./Core/lwip/src/netif/lowpan6_common.cyclo ./Core/lwip/src/netif/lowpan6_common.d ./Core/lwip/src/netif/lowpan6_common.o ./Core/lwip/src/netif/lowpan6_common.su ./Core/lwip/src/netif/slipif.cyclo ./Core/lwip/src/netif/slipif.d ./Core/lwip/src/netif/slipif.o ./Core/lwip/src/netif/slipif.su ./Core/lwip/src/netif/zepif.cyclo ./Core/lwip/src/netif/zepif.d ./Core/lwip/src/netif/zepif.o ./Core/lwip/src/netif/zepif.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-netif

