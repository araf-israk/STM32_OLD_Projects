################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Ra_02_SX1278_lora_stm32f411_TEST
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Ra_02_SX1278_lora_stm32f411_TEST.elf \

MAP_FILES += \
Ra_02_SX1278_lora_stm32f411_TEST.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Ra_02_SX1278_lora_stm32f411_TEST.list \


# All Target
all: main-build

# Main-build Target
main-build: Ra_02_SX1278_lora_stm32f411_TEST.elf secondary-outputs

# Tool invocations
Ra_02_SX1278_lora_stm32f411_TEST.elf Ra_02_SX1278_lora_stm32f411_TEST.map: $(OBJS) $(USER_OBJS) D:\STM32Projects\Ra_02_SX1278_lora_stm32f411_TEST\STM32F411CEUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Ra_02_SX1278_lora_stm32f411_TEST.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\STM32Projects\Ra_02_SX1278_lora_stm32f411_TEST\STM32F411CEUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Ra_02_SX1278_lora_stm32f411_TEST.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Ra_02_SX1278_lora_stm32f411_TEST.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Ra_02_SX1278_lora_stm32f411_TEST.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Ra_02_SX1278_lora_stm32f411_TEST.elf Ra_02_SX1278_lora_stm32f411_TEST.list Ra_02_SX1278_lora_stm32f411_TEST.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
