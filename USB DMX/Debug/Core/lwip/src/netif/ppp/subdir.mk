################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lwip/src/netif/ppp/auth.c \
../Core/lwip/src/netif/ppp/ccp.c \
../Core/lwip/src/netif/ppp/chap-md5.c \
../Core/lwip/src/netif/ppp/chap-new.c \
../Core/lwip/src/netif/ppp/chap_ms.c \
../Core/lwip/src/netif/ppp/demand.c \
../Core/lwip/src/netif/ppp/eap.c \
../Core/lwip/src/netif/ppp/ecp.c \
../Core/lwip/src/netif/ppp/eui64.c \
../Core/lwip/src/netif/ppp/fsm.c \
../Core/lwip/src/netif/ppp/ipcp.c \
../Core/lwip/src/netif/ppp/ipv6cp.c \
../Core/lwip/src/netif/ppp/lcp.c \
../Core/lwip/src/netif/ppp/magic.c \
../Core/lwip/src/netif/ppp/mppe.c \
../Core/lwip/src/netif/ppp/multilink.c \
../Core/lwip/src/netif/ppp/ppp.c \
../Core/lwip/src/netif/ppp/pppapi.c \
../Core/lwip/src/netif/ppp/pppcrypt.c \
../Core/lwip/src/netif/ppp/pppoe.c \
../Core/lwip/src/netif/ppp/pppol2tp.c \
../Core/lwip/src/netif/ppp/pppos.c \
../Core/lwip/src/netif/ppp/upap.c \
../Core/lwip/src/netif/ppp/utils.c \
../Core/lwip/src/netif/ppp/vj.c 

OBJS += \
./Core/lwip/src/netif/ppp/auth.o \
./Core/lwip/src/netif/ppp/ccp.o \
./Core/lwip/src/netif/ppp/chap-md5.o \
./Core/lwip/src/netif/ppp/chap-new.o \
./Core/lwip/src/netif/ppp/chap_ms.o \
./Core/lwip/src/netif/ppp/demand.o \
./Core/lwip/src/netif/ppp/eap.o \
./Core/lwip/src/netif/ppp/ecp.o \
./Core/lwip/src/netif/ppp/eui64.o \
./Core/lwip/src/netif/ppp/fsm.o \
./Core/lwip/src/netif/ppp/ipcp.o \
./Core/lwip/src/netif/ppp/ipv6cp.o \
./Core/lwip/src/netif/ppp/lcp.o \
./Core/lwip/src/netif/ppp/magic.o \
./Core/lwip/src/netif/ppp/mppe.o \
./Core/lwip/src/netif/ppp/multilink.o \
./Core/lwip/src/netif/ppp/ppp.o \
./Core/lwip/src/netif/ppp/pppapi.o \
./Core/lwip/src/netif/ppp/pppcrypt.o \
./Core/lwip/src/netif/ppp/pppoe.o \
./Core/lwip/src/netif/ppp/pppol2tp.o \
./Core/lwip/src/netif/ppp/pppos.o \
./Core/lwip/src/netif/ppp/upap.o \
./Core/lwip/src/netif/ppp/utils.o \
./Core/lwip/src/netif/ppp/vj.o 

C_DEPS += \
./Core/lwip/src/netif/ppp/auth.d \
./Core/lwip/src/netif/ppp/ccp.d \
./Core/lwip/src/netif/ppp/chap-md5.d \
./Core/lwip/src/netif/ppp/chap-new.d \
./Core/lwip/src/netif/ppp/chap_ms.d \
./Core/lwip/src/netif/ppp/demand.d \
./Core/lwip/src/netif/ppp/eap.d \
./Core/lwip/src/netif/ppp/ecp.d \
./Core/lwip/src/netif/ppp/eui64.d \
./Core/lwip/src/netif/ppp/fsm.d \
./Core/lwip/src/netif/ppp/ipcp.d \
./Core/lwip/src/netif/ppp/ipv6cp.d \
./Core/lwip/src/netif/ppp/lcp.d \
./Core/lwip/src/netif/ppp/magic.d \
./Core/lwip/src/netif/ppp/mppe.d \
./Core/lwip/src/netif/ppp/multilink.d \
./Core/lwip/src/netif/ppp/ppp.d \
./Core/lwip/src/netif/ppp/pppapi.d \
./Core/lwip/src/netif/ppp/pppcrypt.d \
./Core/lwip/src/netif/ppp/pppoe.d \
./Core/lwip/src/netif/ppp/pppol2tp.d \
./Core/lwip/src/netif/ppp/pppos.d \
./Core/lwip/src/netif/ppp/upap.d \
./Core/lwip/src/netif/ppp/utils.d \
./Core/lwip/src/netif/ppp/vj.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lwip/src/netif/ppp/%.o Core/lwip/src/netif/ppp/%.su Core/lwip/src/netif/ppp/%.cyclo: ../Core/lwip/src/netif/ppp/%.c Core/lwip/src/netif/ppp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/lwip/src/include -I../Core/lwip/src/include/lwip -I../lwip/src/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lwip-2f-src-2f-netif-2f-ppp

clean-Core-2f-lwip-2f-src-2f-netif-2f-ppp:
	-$(RM) ./Core/lwip/src/netif/ppp/auth.cyclo ./Core/lwip/src/netif/ppp/auth.d ./Core/lwip/src/netif/ppp/auth.o ./Core/lwip/src/netif/ppp/auth.su ./Core/lwip/src/netif/ppp/ccp.cyclo ./Core/lwip/src/netif/ppp/ccp.d ./Core/lwip/src/netif/ppp/ccp.o ./Core/lwip/src/netif/ppp/ccp.su ./Core/lwip/src/netif/ppp/chap-md5.cyclo ./Core/lwip/src/netif/ppp/chap-md5.d ./Core/lwip/src/netif/ppp/chap-md5.o ./Core/lwip/src/netif/ppp/chap-md5.su ./Core/lwip/src/netif/ppp/chap-new.cyclo ./Core/lwip/src/netif/ppp/chap-new.d ./Core/lwip/src/netif/ppp/chap-new.o ./Core/lwip/src/netif/ppp/chap-new.su ./Core/lwip/src/netif/ppp/chap_ms.cyclo ./Core/lwip/src/netif/ppp/chap_ms.d ./Core/lwip/src/netif/ppp/chap_ms.o ./Core/lwip/src/netif/ppp/chap_ms.su ./Core/lwip/src/netif/ppp/demand.cyclo ./Core/lwip/src/netif/ppp/demand.d ./Core/lwip/src/netif/ppp/demand.o ./Core/lwip/src/netif/ppp/demand.su ./Core/lwip/src/netif/ppp/eap.cyclo ./Core/lwip/src/netif/ppp/eap.d ./Core/lwip/src/netif/ppp/eap.o ./Core/lwip/src/netif/ppp/eap.su ./Core/lwip/src/netif/ppp/ecp.cyclo ./Core/lwip/src/netif/ppp/ecp.d ./Core/lwip/src/netif/ppp/ecp.o ./Core/lwip/src/netif/ppp/ecp.su ./Core/lwip/src/netif/ppp/eui64.cyclo ./Core/lwip/src/netif/ppp/eui64.d ./Core/lwip/src/netif/ppp/eui64.o ./Core/lwip/src/netif/ppp/eui64.su ./Core/lwip/src/netif/ppp/fsm.cyclo ./Core/lwip/src/netif/ppp/fsm.d ./Core/lwip/src/netif/ppp/fsm.o ./Core/lwip/src/netif/ppp/fsm.su ./Core/lwip/src/netif/ppp/ipcp.cyclo ./Core/lwip/src/netif/ppp/ipcp.d ./Core/lwip/src/netif/ppp/ipcp.o ./Core/lwip/src/netif/ppp/ipcp.su ./Core/lwip/src/netif/ppp/ipv6cp.cyclo ./Core/lwip/src/netif/ppp/ipv6cp.d ./Core/lwip/src/netif/ppp/ipv6cp.o ./Core/lwip/src/netif/ppp/ipv6cp.su ./Core/lwip/src/netif/ppp/lcp.cyclo ./Core/lwip/src/netif/ppp/lcp.d ./Core/lwip/src/netif/ppp/lcp.o ./Core/lwip/src/netif/ppp/lcp.su ./Core/lwip/src/netif/ppp/magic.cyclo ./Core/lwip/src/netif/ppp/magic.d ./Core/lwip/src/netif/ppp/magic.o ./Core/lwip/src/netif/ppp/magic.su ./Core/lwip/src/netif/ppp/mppe.cyclo ./Core/lwip/src/netif/ppp/mppe.d ./Core/lwip/src/netif/ppp/mppe.o ./Core/lwip/src/netif/ppp/mppe.su ./Core/lwip/src/netif/ppp/multilink.cyclo ./Core/lwip/src/netif/ppp/multilink.d ./Core/lwip/src/netif/ppp/multilink.o ./Core/lwip/src/netif/ppp/multilink.su ./Core/lwip/src/netif/ppp/ppp.cyclo ./Core/lwip/src/netif/ppp/ppp.d ./Core/lwip/src/netif/ppp/ppp.o ./Core/lwip/src/netif/ppp/ppp.su ./Core/lwip/src/netif/ppp/pppapi.cyclo ./Core/lwip/src/netif/ppp/pppapi.d ./Core/lwip/src/netif/ppp/pppapi.o ./Core/lwip/src/netif/ppp/pppapi.su ./Core/lwip/src/netif/ppp/pppcrypt.cyclo ./Core/lwip/src/netif/ppp/pppcrypt.d ./Core/lwip/src/netif/ppp/pppcrypt.o ./Core/lwip/src/netif/ppp/pppcrypt.su ./Core/lwip/src/netif/ppp/pppoe.cyclo ./Core/lwip/src/netif/ppp/pppoe.d ./Core/lwip/src/netif/ppp/pppoe.o ./Core/lwip/src/netif/ppp/pppoe.su ./Core/lwip/src/netif/ppp/pppol2tp.cyclo ./Core/lwip/src/netif/ppp/pppol2tp.d ./Core/lwip/src/netif/ppp/pppol2tp.o ./Core/lwip/src/netif/ppp/pppol2tp.su ./Core/lwip/src/netif/ppp/pppos.cyclo ./Core/lwip/src/netif/ppp/pppos.d ./Core/lwip/src/netif/ppp/pppos.o ./Core/lwip/src/netif/ppp/pppos.su ./Core/lwip/src/netif/ppp/upap.cyclo ./Core/lwip/src/netif/ppp/upap.d ./Core/lwip/src/netif/ppp/upap.o ./Core/lwip/src/netif/ppp/upap.su ./Core/lwip/src/netif/ppp/utils.cyclo ./Core/lwip/src/netif/ppp/utils.d ./Core/lwip/src/netif/ppp/utils.o ./Core/lwip/src/netif/ppp/utils.su ./Core/lwip/src/netif/ppp/vj.cyclo ./Core/lwip/src/netif/ppp/vj.d ./Core/lwip/src/netif/ppp/vj.o ./Core/lwip/src/netif/ppp/vj.su

.PHONY: clean-Core-2f-lwip-2f-src-2f-netif-2f-ppp

