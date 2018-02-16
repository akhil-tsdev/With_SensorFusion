################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include platform/wdog/hal/subdir.mk
-include platform/wdog/drivers/subdir.mk
-include platform/wdog/common/subdir.mk
-include platform/uart/hal/subdir.mk
-include platform/uart/drivers/common/subdir.mk
-include platform/uart/drivers/subdir.mk
-include platform/smc/hal/subdir.mk
-include platform/sim/hal/subdir.mk
-include platform/sai/hal/subdir.mk
-include platform/sai/drivers/subdir.mk
-include platform/sai/common/subdir.mk
-include platform/rtc/hal/subdir.mk
-include platform/rtc/drivers/subdir.mk
-include platform/rcm/hal/subdir.mk
-include platform/power/system/subdir.mk
-include platform/port/hal/subdir.mk
-include platform/pmc/hal/subdir.mk
-include platform/pit/hal/subdir.mk
-include platform/pit/drivers/subdir.mk
-include platform/pit/common/subdir.mk
-include platform/pdb/hal/subdir.mk
-include platform/pdb/drivers/subdir.mk
-include platform/pdb/common/subdir.mk
-include platform/osc/hal/subdir.mk
-include platform/mcg/hal/subdir.mk
-include platform/lpuart/hal/subdir.mk
-include platform/lpuart/drivers/subdir.mk
-include platform/lpuart/common/subdir.mk
-include platform/lptmr/hal/subdir.mk
-include platform/lptmr/drivers/subdir.mk
-include platform/lptmr/common/subdir.mk
-include platform/llwu/hal/subdir.mk
-include platform/interrupt/system/subdir.mk
-include platform/i2c/hal/subdir.mk
-include platform/i2c/drivers/slave/subdir.mk
-include platform/i2c/drivers/master/subdir.mk
-include platform/i2c/drivers/common/subdir.mk
-include platform/hwtimer/system/subdir.mk
-include platform/gpio/hal/subdir.mk
-include platform/gpio/drivers/subdir.mk
-include platform/gpio/common/subdir.mk
-include platform/flextimer/hal/subdir.mk
-include platform/flextimer/drivers/subdir.mk
-include platform/edma/hal/subdir.mk
-include platform/edma/drivers/subdir.mk
-include platform/edma/common/subdir.mk
-include platform/dspi/hal/subdir.mk
-include platform/dspi/drivers/slave/subdir.mk
-include platform/dspi/drivers/master/subdir.mk
-include platform/dspi/drivers/common/subdir.mk
-include platform/dmamux/hal/subdir.mk
-include platform/dac/hal/subdir.mk
-include platform/dac/drivers/subdir.mk
-include platform/dac/common/subdir.mk
-include platform/clock/system/subdir.mk
-include platform/adc/hal/subdir.mk
-include platform/adc/drivers/subdir.mk
-include platform/adc/common/subdir.mk
-include osa/subdir.mk
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
all: pre-build main-build

# Main-build Target
main-build: C:/Users/Chamdani/Desktop/github/trunk/wearable/22F_sensor_module/lib/ksdk_platform_lib_K22F12810/release/ksdk_platform_lib.a

# Tool invocations
C:/Users/Chamdani/Desktop/github/trunk/wearable/22F_sensor_module/lib/ksdk_platform_lib_K22F12810/release/ksdk_platform_lib.a: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU Archiver'
	arm-none-eabi-ar -r  "C:/Users/Chamdani/Desktop/github/trunk/wearable/22F_sensor_module/lib/ksdk_platform_lib_K22F12810/release/ksdk_platform_lib.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(ARCHIVES)$(S_UPPER_DEPS) C:/Users/Chamdani/Desktop/github/trunk/wearable/22F_sensor_module/lib/ksdk_platform_lib_K22F12810/release/ksdk_platform_lib.a
	-@echo ' '

pre-build:
	-C:/Users/Chamdani/Desktop/github/trunk/wearable/22F_sensor_module/lib/ksdk_platform_lib_K22F12810/makedir.bat release
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: main-build pre-build

-include ../makefile.targets
