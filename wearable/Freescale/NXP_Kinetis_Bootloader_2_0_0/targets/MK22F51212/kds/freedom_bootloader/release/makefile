################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/utilities/src/subdir.mk
-include src/usb/osa/subdir.mk
-include src/usb/device/calss/subdir.mk
-include src/usb/device/subdir.mk
-include src/usb/bm_composite/subdir.mk
-include src/startup/subdir.mk
-include src/sbloader/src/subdir.mk
-include src/property/src/subdir.mk
-include src/packet/src/subdir.mk
-include src/memory/src/subdir.mk
-include src/drivers/uart/subdir.mk
-include src/drivers/smc/subdir.mk
-include src/drivers/microseconds/src/subdir.mk
-include src/drivers/i2c/src/subdir.mk
-include src/drivers/gpio/subdir.mk
-include src/drivers/flash/subdir.mk
-include src/drivers/dspi/subdir.mk
-include src/drivers/crc/subdir.mk
-include src/crc/src/subdir.mk
-include src/bootloader/src/subdir.mk
-include src/autobaud/src/subdir.mk
-include MK22F51212/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: freedom_bootloader.elf secondary-outputs

# Tool invocations
freedom_bootloader.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -T ../../../../common/linker/kds/MK22F51212/MK22FN512xxx12_application_0x0000.ld -Xlinker --gc-sections -Wl,-Map,"freedom_bootloader.map" -Xlinker -z  -Xlinker muldefs   -Os  --specs=nano.specs          -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -fshort-wchar -Wl,--no-wchar-size-warning  -mapcs  -Xlinker -static  -flto  -Xlinker --defsym=__ram_vector_table__=1 -o "freedom_bootloader.elf" -Xlinker --start-group $(OBJS) $(USER_OBJS) $(LIBS) -Xlinker --end-group
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(S_UPPER_DEPS) freedom_bootloader.elf
	-@echo ' '

secondary-outputs:

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
