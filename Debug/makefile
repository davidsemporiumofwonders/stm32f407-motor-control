################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
motor_controller.hex \

SECONDARY_LIST += \
motor_controller.lst \

SECONDARY_SIZE += \
motor_controller.siz \


# All Target
all: motor_controller.elf secondary-outputs

# Tool invocations
motor_controller.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM Cross C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -march=armv7e-m -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -munaligned-access -O3 -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -fsingle-precision-constant -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Wfloat-equal  -g3 -ggdb -T "/home/david/Documents/workspace-arm/motor_controller/linker.ld" -nostdlib -Xlinker --gc-sections -Wl,-Map,"motor_controller.map" --specs=nano.specs -o "motor_controller.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

motor_controller.hex: motor_controller.elf
	@echo 'Invoking: GNU ARM Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "motor_controller.elf"  "motor_controller.hex"
	@echo 'Finished building: $@'
	@echo ' '

motor_controller.lst: motor_controller.elf
	@echo 'Invoking: GNU ARM Cross Create Listing'
	arm-none-eabi-objdump --source --all-headers --demangle --line-numbers --wide "motor_controller.elf" > "motor_controller.lst"
	@echo 'Finished building: $@'
	@echo ' '

motor_controller.siz: motor_controller.elf
	@echo 'Invoking: GNU ARM Cross Print Size'
	arm-none-eabi-size --format=berkeley "motor_controller.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) motor_controller.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
