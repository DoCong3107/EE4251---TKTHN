################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/LoRa_lib/subdir.mk
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


BUILD_ARTIFACT_NAME := Test02_LoRa_STM32F103_Relay_Register
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Test02_LoRa_STM32F103_Relay_Register.elf \

MAP_FILES += \
Test02_LoRa_STM32F103_Relay_Register.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Test02_LoRa_STM32F103_Relay_Register.list \

OBJCOPY_HEX += \
Test02_LoRa_STM32F103_Relay_Register.hex \


# All Target
all: main-build

# Main-build Target
main-build: Test02_LoRa_STM32F103_Relay_Register.elf secondary-outputs

# Tool invocations
Test02_LoRa_STM32F103_Relay_Register.elf Test02_LoRa_STM32F103_Relay_Register.map: $(OBJS) $(USER_OBJS) C:\STM32IDE\Test02_LoRa_STM32F103_Relay_Register\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Test02_LoRa_STM32F103_Relay_Register.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\STM32IDE\Test02_LoRa_STM32F103_Relay_Register\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Test02_LoRa_STM32F103_Relay_Register.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Test02_LoRa_STM32F103_Relay_Register.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Test02_LoRa_STM32F103_Relay_Register.list"
	@echo 'Finished building: $@'
	@echo ' '

Test02_LoRa_STM32F103_Relay_Register.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Test02_LoRa_STM32F103_Relay_Register.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Test02_LoRa_STM32F103_Relay_Register.elf Test02_LoRa_STM32F103_Relay_Register.hex Test02_LoRa_STM32F103_Relay_Register.list Test02_LoRa_STM32F103_Relay_Register.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
