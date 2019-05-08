################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main.c 

S_UPPER_SRCS += \
../asm_functions.S \
../startup.S 

OBJS += \
./asm_functions.o \
./main.o \
./startup.o 

S_UPPER_DEPS += \
./asm_functions.d \
./startup.d 

C_DEPS += \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -march=armv7e-m -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -munaligned-access -O3 -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -fsingle-precision-constant -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Wfloat-equal  -g3 -ggdb -x assembler-with-cpp -DSTM32F40_41xxx -I/home/david/mcutoolchainsandlibraries/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/Libraries/CMSIS/Include -I/home/david/mcutoolchainsandlibraries/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/Libraries/STM32F4xx_StdPeriph_Driver/inc -I/home/david/mcutoolchainsandlibraries/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/Libraries/CMSIS/Device/ST/STM32F4xx/Include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -march=armv7e-m -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -munaligned-access -O3 -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -fsingle-precision-constant -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Wfloat-equal  -g3 -ggdb -DSTM32F40_41xxx -I/home/david/mcutoolchainsandlibraries/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/Libraries/CMSIS/Include -I/home/david/mcutoolchainsandlibraries/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/Libraries/STM32F4xx_StdPeriph_Driver/inc -I/home/david/mcutoolchainsandlibraries/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/Libraries/CMSIS/Device/ST/STM32F4xx/Include -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


