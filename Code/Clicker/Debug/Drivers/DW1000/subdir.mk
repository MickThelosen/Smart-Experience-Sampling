################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DW1000/DW1000.c \
../Drivers/DW1000/SPI.c 

OBJS += \
./Drivers/DW1000/DW1000.o \
./Drivers/DW1000/SPI.o 

C_DEPS += \
./Drivers/DW1000/DW1000.d \
./Drivers/DW1000/SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DW1000/%.o Drivers/DW1000/%.su Drivers/DW1000/%.cyclo: ../Drivers/DW1000/%.c Drivers/DW1000/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB07 -c -I../Core/Inc -I../Drivers/STM32WB0x_HAL_Driver/Inc -I../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../Drivers/CMSIS/Include -I"C:/Users/mick/Documents/Github/embedded-clicker/Clicker/Drivers/DW1000" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-DW1000

clean-Drivers-2f-DW1000:
	-$(RM) ./Drivers/DW1000/DW1000.cyclo ./Drivers/DW1000/DW1000.d ./Drivers/DW1000/DW1000.o ./Drivers/DW1000/DW1000.su ./Drivers/DW1000/SPI.cyclo ./Drivers/DW1000/SPI.d ./Drivers/DW1000/SPI.o ./Drivers/DW1000/SPI.su

.PHONY: clean-Drivers-2f-DW1000

