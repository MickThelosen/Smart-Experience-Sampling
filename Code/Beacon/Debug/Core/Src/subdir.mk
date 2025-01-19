################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Src/blue_unit_conversion.s 

C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32wb0x_hal_msp.c \
../Core/Src/stm32wb0x_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wb0x.c 

OBJS += \
./Core/Src/blue_unit_conversion.o \
./Core/Src/main.o \
./Core/Src/stm32wb0x_hal_msp.o \
./Core/Src/stm32wb0x_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wb0x.o 

S_DEPS += \
./Core/Src/blue_unit_conversion.d 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32wb0x_hal_msp.d \
./Core/Src/stm32wb0x_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wb0x.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.s Core/Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB07 -c -I../Core/Inc -I../Drivers/STM32WB0x_HAL_Driver/Inc -I../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../Drivers/CMSIS/Include -I"C:/Users/mick/Documents/Github/embedded-clicker/Beacon/Drivers/DW1000" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/blue_unit_conversion.d ./Core/Src/blue_unit_conversion.o ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32wb0x_hal_msp.cyclo ./Core/Src/stm32wb0x_hal_msp.d ./Core/Src/stm32wb0x_hal_msp.o ./Core/Src/stm32wb0x_hal_msp.su ./Core/Src/stm32wb0x_it.cyclo ./Core/Src/stm32wb0x_it.d ./Core/Src/stm32wb0x_it.o ./Core/Src/stm32wb0x_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wb0x.cyclo ./Core/Src/system_stm32wb0x.d ./Core/Src/system_stm32wb0x.o ./Core/Src/system_stm32wb0x.su

.PHONY: clean-Core-2f-Src

