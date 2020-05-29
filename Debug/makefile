################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F2xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
GEN\ Charger\ Firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
GEN\ Charger\ Firmware.list \

OBJCOPY_BIN += \
GEN\ Charger\ Firmware.bin \


# All Target
all: GEN\ Charger\ Firmware.elf secondary-outputs

# Tool invocations
GEN\ Charger\ Firmware.elf: $(OBJS) $(USER_OBJS) C:\Users\faiz\git\GEN_Charger\STM32F205RETX_FLASH.ld
	arm-none-eabi-gcc -o "GEN Charger Firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\faiz\git\GEN_Charger\STM32F205RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="GEN Charger Firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

GEN\ Charger\ Firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "GEN Charger Firmware.list"
	@echo 'Finished building: $@'
	@echo ' '

GEN\ Charger\ Firmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "GEN Charger Firmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
