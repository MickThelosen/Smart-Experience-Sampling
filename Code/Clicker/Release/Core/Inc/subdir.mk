################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/DW1000.c \
../Core/Inc/SPI.c 

OBJS += \
./Core/Inc/DW1000.o \
./Core/Inc/SPI.o 

C_DEPS += \
./Core/Inc/DW1000.d \
./Core/Inc/SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32WB07 -c -I../Core/Inc -I"C:/Users/mick/Documents/Github/embedded-clicker/Clicker/Drivers" -I../Drivers/STM32WB0x_HAL_Driver/Inc -I../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/DW1000.cyclo ./Core/Inc/DW1000.d ./Core/Inc/DW1000.o ./Core/Inc/DW1000.su ./Core/Inc/SPI.cyclo ./Core/Inc/SPI.d ./Core/Inc/SPI.o ./Core/Inc/SPI.su

.PHONY: clean-Core-2f-Inc

