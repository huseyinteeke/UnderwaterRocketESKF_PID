################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Thirdparty/SEGGER/SEGGER/Syscalls/subdir.mk
-include Thirdparty/SEGGER/SEGGER/subdir.mk
-include Thirdparty/SEGGER/Rec/subdir.mk
-include Thirdparty/SEGGER/OS/subdir.mk
-include Thirdparty/SEGGER/Config/subdir.mk
-include Thirdparty/FreeRTOS/portable/MemMang/subdir.mk
-include Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F/subdir.mk
-include Thirdparty/FreeRTOS/subdir.mk
-include Tasks/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Algorithm+Sensors/PID/subdir.mk
-include Algorithm+Sensors/MS5837/subdir.mk
-include Algorithm+Sensors/MAESTRO/subdir.mk
-include Algorithm+Sensors/BNO055/subdir.mk
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


BUILD_ARTIFACT_NAME := underwater_rocket
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
underwater_rocket.elf \

MAP_FILES += \
underwater_rocket.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
underwater_rocket.list \


# All Target
all: main-build

# Main-build Target
main-build: underwater_rocket.elf secondary-outputs

# Tool invocations
underwater_rocket.elf underwater_rocket.map: $(OBJS) $(USER_OBJS) C:\Users\husey\OneDrive\Masaüstü\Underwater_rocket\RTOS_workspace\underwater_rocket\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "underwater_rocket.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\husey\OneDrive\Masaüstü\Underwater_rocket\RTOS_workspace\underwater_rocket\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="underwater_rocket.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

underwater_rocket.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "underwater_rocket.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout underwater_rocket.elf underwater_rocket.list underwater_rocket.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
