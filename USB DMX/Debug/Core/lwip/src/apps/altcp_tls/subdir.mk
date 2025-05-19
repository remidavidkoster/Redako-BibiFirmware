################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.c \
../Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.c 

OBJS += \
./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.o \
./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.o 

C_DEPS += \
./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.d \
./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/apps/altcp_tls/%.o Core/lwip/src/apps/altcp_tls/%.su Core/lwip/src/apps/altcp_tls/%.cyclo: ../Core/lwip/src/apps/altcp_tls/%.c Core/lwip/src/apps/altcp_tls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-apps-2f-altcp_tls

clean-Core-2f-lwip-2f-src-2f-apps-2f-altcp_tls:
	-$(RM) ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.cyclo ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.d ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.o ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls.su ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.cyclo ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.d ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.o ./Core/lwip/src/apps/altcp_tls/altcp_tls_mbedtls_mem.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-apps-2f-altcp_tls

