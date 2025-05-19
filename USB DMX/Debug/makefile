################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/lwip/src/netif/ppp/polarssl/subdir.mk
-include Core/lwip/src/netif/ppp/subdir.mk
-include Core/lwip/src/netif/subdir.mk
-include Core/lwip/src/core/ipv6/subdir.mk
-include Core/lwip/src/core/ipv4/subdir.mk
-include Core/lwip/src/core/subdir.mk
-include Core/lwip/src/apps/snmp/subdir.mk
-include Core/lwip/src/apps/mdns/subdir.mk
-include Core/lwip/src/apps/lwiperf/subdir.mk
-include Core/lwip/src/apps/http/subdir.mk
-include Core/lwip/src/api/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/oled/subdir.mk
-include Core/Src/eth/subdir.mk
-include Core/Src/subdir.mk
-include Core/Inc/eth/fs/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := USB DMX
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
USB\ DMX.elf \

MAP_FILES += \
USB\ DMX.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
USB\ DMX.list \


# All Target
all: main-build

# Main-build Target
main-build: USB\ DMX.elf secondary-outputs

# Tool invocations
USB\ DMX.elf USB\ DMX.map: $(OBJS) $(USER_OBJS) C:\Git\Redako-BibiFirmware\USB\ DMX\STM32G431CBUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "USB DMX.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Git\Redako-BibiFirmware\USB DMX\STM32G431CBUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="USB DMX.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

USB\ DMX.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "USB DMX.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) USB\ DMX.elf USB\ DMX.list USB\ DMX.map USB\\ DMX.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
