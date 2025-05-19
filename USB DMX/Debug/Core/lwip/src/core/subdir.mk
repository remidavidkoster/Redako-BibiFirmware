################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/core/altcp.c \
../Core/lwip/src/core/altcp_alloc.c \
../Core/lwip/src/core/altcp_tcp.c \
../Core/lwip/src/core/def.c \
../Core/lwip/src/core/dns.c \
../Core/lwip/src/core/inet_chksum.c \
../Core/lwip/src/core/init.c \
../Core/lwip/src/core/ip.c \
../Core/lwip/src/core/mem.c \
../Core/lwip/src/core/memp.c \
../Core/lwip/src/core/netif.c \
../Core/lwip/src/core/pbuf.c \
../Core/lwip/src/core/raw.c \
../Core/lwip/src/core/stats.c \
../Core/lwip/src/core/sys.c \
../Core/lwip/src/core/tcp.c \
../Core/lwip/src/core/tcp_in.c \
../Core/lwip/src/core/tcp_out.c \
../Core/lwip/src/core/timeouts.c \
../Core/lwip/src/core/udp.c 

OBJS += \
./Core/lwip/src/core/altcp.o \
./Core/lwip/src/core/altcp_alloc.o \
./Core/lwip/src/core/altcp_tcp.o \
./Core/lwip/src/core/def.o \
./Core/lwip/src/core/dns.o \
./Core/lwip/src/core/inet_chksum.o \
./Core/lwip/src/core/init.o \
./Core/lwip/src/core/ip.o \
./Core/lwip/src/core/mem.o \
./Core/lwip/src/core/memp.o \
./Core/lwip/src/core/netif.o \
./Core/lwip/src/core/pbuf.o \
./Core/lwip/src/core/raw.o \
./Core/lwip/src/core/stats.o \
./Core/lwip/src/core/sys.o \
./Core/lwip/src/core/tcp.o \
./Core/lwip/src/core/tcp_in.o \
./Core/lwip/src/core/tcp_out.o \
./Core/lwip/src/core/timeouts.o \
./Core/lwip/src/core/udp.o 

C_DEPS += \
./Core/lwip/src/core/altcp.d \
./Core/lwip/src/core/altcp_alloc.d \
./Core/lwip/src/core/altcp_tcp.d \
./Core/lwip/src/core/def.d \
./Core/lwip/src/core/dns.d \
./Core/lwip/src/core/inet_chksum.d \
./Core/lwip/src/core/init.d \
./Core/lwip/src/core/ip.d \
./Core/lwip/src/core/mem.d \
./Core/lwip/src/core/memp.d \
./Core/lwip/src/core/netif.d \
./Core/lwip/src/core/pbuf.d \
./Core/lwip/src/core/raw.d \
./Core/lwip/src/core/stats.d \
./Core/lwip/src/core/sys.d \
./Core/lwip/src/core/tcp.d \
./Core/lwip/src/core/tcp_in.d \
./Core/lwip/src/core/tcp_out.d \
./Core/lwip/src/core/timeouts.d \
./Core/lwip/src/core/udp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/core/%.o Core/lwip/src/core/%.su Core/lwip/src/core/%.cyclo: ../Core/lwip/src/core/%.c Core/lwip/src/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include/ -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-core

clean-Core-2f-lwip-2f-src-2f-core:
	-$(RM) ./Core/lwip/src/core/altcp.cyclo ./Core/lwip/src/core/altcp.d ./Core/lwip/src/core/altcp.o ./Core/lwip/src/core/altcp.su ./Core/lwip/src/core/altcp_alloc.cyclo ./Core/lwip/src/core/altcp_alloc.d ./Core/lwip/src/core/altcp_alloc.o ./Core/lwip/src/core/altcp_alloc.su ./Core/lwip/src/core/altcp_tcp.cyclo ./Core/lwip/src/core/altcp_tcp.d ./Core/lwip/src/core/altcp_tcp.o ./Core/lwip/src/core/altcp_tcp.su ./Core/lwip/src/core/def.cyclo ./Core/lwip/src/core/def.d ./Core/lwip/src/core/def.o ./Core/lwip/src/core/def.su ./Core/lwip/src/core/dns.cyclo ./Core/lwip/src/core/dns.d ./Core/lwip/src/core/dns.o ./Core/lwip/src/core/dns.su ./Core/lwip/src/core/inet_chksum.cyclo ./Core/lwip/src/core/inet_chksum.d ./Core/lwip/src/core/inet_chksum.o ./Core/lwip/src/core/inet_chksum.su ./Core/lwip/src/core/init.cyclo ./Core/lwip/src/core/init.d ./Core/lwip/src/core/init.o ./Core/lwip/src/core/init.su ./Core/lwip/src/core/ip.cyclo ./Core/lwip/src/core/ip.d ./Core/lwip/src/core/ip.o ./Core/lwip/src/core/ip.su ./Core/lwip/src/core/mem.cyclo ./Core/lwip/src/core/mem.d ./Core/lwip/src/core/mem.o ./Core/lwip/src/core/mem.su ./Core/lwip/src/core/memp.cyclo ./Core/lwip/src/core/memp.d ./Core/lwip/src/core/memp.o ./Core/lwip/src/core/memp.su ./Core/lwip/src/core/netif.cyclo ./Core/lwip/src/core/netif.d ./Core/lwip/src/core/netif.o ./Core/lwip/src/core/netif.su ./Core/lwip/src/core/pbuf.cyclo ./Core/lwip/src/core/pbuf.d ./Core/lwip/src/core/pbuf.o ./Core/lwip/src/core/pbuf.su ./Core/lwip/src/core/raw.cyclo ./Core/lwip/src/core/raw.d ./Core/lwip/src/core/raw.o ./Core/lwip/src/core/raw.su ./Core/lwip/src/core/stats.cyclo ./Core/lwip/src/core/stats.d ./Core/lwip/src/core/stats.o ./Core/lwip/src/core/stats.su ./Core/lwip/src/core/sys.cyclo ./Core/lwip/src/core/sys.d ./Core/lwip/src/core/sys.o ./Core/lwip/src/core/sys.su ./Core/lwip/src/core/tcp.cyclo ./Core/lwip/src/core/tcp.d ./Core/lwip/src/core/tcp.o ./Core/lwip/src/core/tcp.su ./Core/lwip/src/core/tcp_in.cyclo ./Core/lwip/src/core/tcp_in.d ./Core/lwip/src/core/tcp_in.o ./Core/lwip/src/core/tcp_in.su ./Core/lwip/src/core/tcp_out.cyclo ./Core/lwip/src/core/tcp_out.d ./Core/lwip/src/core/tcp_out.o ./Core/lwip/src/core/tcp_out.su ./Core/lwip/src/core/timeouts.cyclo ./Core/lwip/src/core/timeouts.d ./Core/lwip/src/core/timeouts.o ./Core/lwip/src/core/timeouts.su ./Core/lwip/src/core/udp.cyclo ./Core/lwip/src/core/udp.d ./Core/lwip/src/core/udp.o ./Core/lwip/src/core/udp.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-core

