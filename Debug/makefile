################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Strain Gauges - CubeMX
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Strain\ Gauges\ -\ CubeMX.elf \

MAP_FILES += \
Strain\ Gauges\ -\ CubeMX.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Strain\ Gauges\ -\ CubeMX.list \


# All Target
all: main-build

# Main-build Target
main-build: Strain\ Gauges\ -\ CubeMX.elf secondary-outputs

# Tool invocations
Strain\ Gauges\ -\ CubeMX.elf Strain\ Gauges\ -\ CubeMX.map: $(OBJS) $(USER_OBJS) C:\Users\Nikos\Desktop\UTh\Centaurus\Projects\Strain\ Gauges\ OpAmps\Codes\Strain\ Gauges\ -\ CubeMX\STM32F103CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Strain Gauges - CubeMX.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Nikos\Desktop\UTh\Centaurus\Projects\Strain Gauges OpAmps\Codes\Strain Gauges - CubeMX\STM32F103CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Strain Gauges - CubeMX.map" -Wl,--gc-sections -static -LMAX6675.h --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Strain\ Gauges\ -\ CubeMX.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Strain Gauges - CubeMX.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Strain\ Gauges\ -\ CubeMX.elf Strain\ Gauges\ -\ CubeMX.list Strain\ Gauges\ -\ CubeMX.map Strain\\ Gauges\\ -\\ CubeMX.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
