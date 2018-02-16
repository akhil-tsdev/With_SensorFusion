################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/startup/subdir.mk
-include MK22F25612/subdir.mk
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
SECONDARY_FLASH += \
led_demo_tower_a000_turingsense.bin \


# All Target
all: led_demo_tower_a000_turingsense.elf secondary-outputs

# Tool invocations
led_demo_tower_a000_turingsense.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -w -T "C:\Users\cwati\Documents\My Work\Freescale\Bootloader\NXP_Kinetis_Bootloader_2_0_0\NXP_Kinetis_Bootloader_2_0_0\targets\common\linker\kds\MK22F25612\MK22FN256xxx12_application_0xa000.ld" -Xlinker --gc-sections -Wl,-Map,"led_demo_tower_a000_turingsense.map" -Xlinker -z  -Xlinker muldefs   -Os  --specs=nano.specs          -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -Xlinker -static  -Xlinker --defsym=__ram_vector_table__=1 -o "led_demo_tower_a000_turingsense.elf" -Xlinker --start-group $(OBJS) $(USER_OBJS) $(LIBS) -Xlinker --end-group
	@echo 'Finished building target: $@'
	@echo ' '

led_demo_tower_a000_turingsense.bin: led_demo_tower_a000_turingsense.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O binary "led_demo_tower_a000_turingsense.elf"  "led_demo_tower_a000_turingsense.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(S_UPPER_DEPS) led_demo_tower_a000_turingsense.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
