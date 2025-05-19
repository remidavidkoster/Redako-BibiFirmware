################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/api/api_lib.c \
../Core/lwip/src/api/api_msg.c \
../Core/lwip/src/api/err.c \
../Core/lwip/src/api/if_api.c \
../Core/lwip/src/api/netbuf.c \
../Core/lwip/src/api/netdb.c \
../Core/lwip/src/api/netifapi.c \
../Core/lwip/src/api/sockets.c \
../Core/lwip/src/api/tcpip.c 

OBJS += \
./Core/lwip/src/api/api_lib.o \
./Core/lwip/src/api/api_msg.o \
./Core/lwip/src/api/err.o \
./Core/lwip/src/api/if_api.o \
./Core/lwip/src/api/netbuf.o \
./Core/lwip/src/api/netdb.o \
./Core/lwip/src/api/netifapi.o \
./Core/lwip/src/api/sockets.o \
./Core/lwip/src/api/tcpip.o 

C_DEPS += \
./Core/lwip/src/api/api_lib.d \
./Core/lwip/src/api/api_msg.d \
./Core/lwip/src/api/err.d \
./Core/lwip/src/api/if_api.d \
./Core/lwip/src/api/netbuf.d \
./Core/lwip/src/api/netdb.d \
./Core/lwip/src/api/netifapi.d \
./Core/lwip/src/api/sockets.d \
./Core/lwip/src/api/tcpip.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/api/%.o Core/lwip/src/api/%.su Core/lwip/src/api/%.cyclo: ../Core/lwip/src/api/%.c Core/lwip/src/api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-api

clean-Core-2f-lwip-2f-src-2f-api:
	-$(RM) ./Core/lwip/src/api/api_lib.cyclo ./Core/lwip/src/api/api_lib.d ./Core/lwip/src/api/api_lib.o ./Core/lwip/src/api/api_lib.su ./Core/lwip/src/api/api_msg.cyclo ./Core/lwip/src/api/api_msg.d ./Core/lwip/src/api/api_msg.o ./Core/lwip/src/api/api_msg.su ./Core/lwip/src/api/err.cyclo ./Core/lwip/src/api/err.d ./Core/lwip/src/api/err.o ./Core/lwip/src/api/err.su ./Core/lwip/src/api/if_api.cyclo ./Core/lwip/src/api/if_api.d ./Core/lwip/src/api/if_api.o ./Core/lwip/src/api/if_api.su ./Core/lwip/src/api/netbuf.cyclo ./Core/lwip/src/api/netbuf.d ./Core/lwip/src/api/netbuf.o ./Core/lwip/src/api/netbuf.su ./Core/lwip/src/api/netdb.cyclo ./Core/lwip/src/api/netdb.d ./Core/lwip/src/api/netdb.o ./Core/lwip/src/api/netdb.su ./Core/lwip/src/api/netifapi.cyclo ./Core/lwip/src/api/netifapi.d ./Core/lwip/src/api/netifapi.o ./Core/lwip/src/api/netifapi.su ./Core/lwip/src/api/sockets.cyclo ./Core/lwip/src/api/sockets.d ./Core/lwip/src/api/sockets.o ./Core/lwip/src/api/sockets.su ./Core/lwip/src/api/tcpip.cyclo ./Core/lwip/src/api/tcpip.d ./Core/lwip/src/api/tcpip.o ./Core/lwip/src/api/tcpip.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-api

