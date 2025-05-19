################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/core/ipv6/dhcp6.c \
../Core/lwip/src/core/ipv6/ethip6.c \
../Core/lwip/src/core/ipv6/icmp6.c \
../Core/lwip/src/core/ipv6/inet6.c \
../Core/lwip/src/core/ipv6/ip6.c \
../Core/lwip/src/core/ipv6/ip6_addr.c \
../Core/lwip/src/core/ipv6/ip6_frag.c \
../Core/lwip/src/core/ipv6/mld6.c \
../Core/lwip/src/core/ipv6/nd6.c 

OBJS += \
./Core/lwip/src/core/ipv6/dhcp6.o \
./Core/lwip/src/core/ipv6/ethip6.o \
./Core/lwip/src/core/ipv6/icmp6.o \
./Core/lwip/src/core/ipv6/inet6.o \
./Core/lwip/src/core/ipv6/ip6.o \
./Core/lwip/src/core/ipv6/ip6_addr.o \
./Core/lwip/src/core/ipv6/ip6_frag.o \
./Core/lwip/src/core/ipv6/mld6.o \
./Core/lwip/src/core/ipv6/nd6.o 

C_DEPS += \
./Core/lwip/src/core/ipv6/dhcp6.d \
./Core/lwip/src/core/ipv6/ethip6.d \
./Core/lwip/src/core/ipv6/icmp6.d \
./Core/lwip/src/core/ipv6/inet6.d \
./Core/lwip/src/core/ipv6/ip6.d \
./Core/lwip/src/core/ipv6/ip6_addr.d \
./Core/lwip/src/core/ipv6/ip6_frag.d \
./Core/lwip/src/core/ipv6/mld6.d \
./Core/lwip/src/core/ipv6/nd6.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/core/ipv6/%.o Core/lwip/src/core/ipv6/%.su Core/lwip/src/core/ipv6/%.cyclo: ../Core/lwip/src/core/ipv6/%.c Core/lwip/src/core/ipv6/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-core-2f-ipv6

clean-Core-2f-lwip-2f-src-2f-core-2f-ipv6:
	-$(RM) ./Core/lwip/src/core/ipv6/dhcp6.cyclo ./Core/lwip/src/core/ipv6/dhcp6.d ./Core/lwip/src/core/ipv6/dhcp6.o ./Core/lwip/src/core/ipv6/dhcp6.su ./Core/lwip/src/core/ipv6/ethip6.cyclo ./Core/lwip/src/core/ipv6/ethip6.d ./Core/lwip/src/core/ipv6/ethip6.o ./Core/lwip/src/core/ipv6/ethip6.su ./Core/lwip/src/core/ipv6/icmp6.cyclo ./Core/lwip/src/core/ipv6/icmp6.d ./Core/lwip/src/core/ipv6/icmp6.o ./Core/lwip/src/core/ipv6/icmp6.su ./Core/lwip/src/core/ipv6/inet6.cyclo ./Core/lwip/src/core/ipv6/inet6.d ./Core/lwip/src/core/ipv6/inet6.o ./Core/lwip/src/core/ipv6/inet6.su ./Core/lwip/src/core/ipv6/ip6.cyclo ./Core/lwip/src/core/ipv6/ip6.d ./Core/lwip/src/core/ipv6/ip6.o ./Core/lwip/src/core/ipv6/ip6.su ./Core/lwip/src/core/ipv6/ip6_addr.cyclo ./Core/lwip/src/core/ipv6/ip6_addr.d ./Core/lwip/src/core/ipv6/ip6_addr.o ./Core/lwip/src/core/ipv6/ip6_addr.su ./Core/lwip/src/core/ipv6/ip6_frag.cyclo ./Core/lwip/src/core/ipv6/ip6_frag.d ./Core/lwip/src/core/ipv6/ip6_frag.o ./Core/lwip/src/core/ipv6/ip6_frag.su ./Core/lwip/src/core/ipv6/mld6.cyclo ./Core/lwip/src/core/ipv6/mld6.d ./Core/lwip/src/core/ipv6/mld6.o ./Core/lwip/src/core/ipv6/mld6.su ./Core/lwip/src/core/ipv6/nd6.cyclo ./Core/lwip/src/core/ipv6/nd6.d ./Core/lwip/src/core/ipv6/nd6.o ./Core/lwip/src/core/ipv6/nd6.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-core-2f-ipv6

