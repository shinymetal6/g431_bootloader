################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Bootloader/bootloader.c \
../Core/Src/Bootloader/flash.c \
../Core/Src/Bootloader/uart.c \
../Core/Src/Bootloader/xmodem.c 

OBJS += \
./Core/Src/Bootloader/bootloader.o \
./Core/Src/Bootloader/flash.o \
./Core/Src/Bootloader/uart.o \
./Core/Src/Bootloader/xmodem.o 

C_DEPS += \
./Core/Src/Bootloader/bootloader.d \
./Core/Src/Bootloader/flash.d \
./Core/Src/Bootloader/uart.d \
./Core/Src/Bootloader/xmodem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Bootloader/%.o: ../Core/Src/Bootloader/%.c Core/Src/Bootloader/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/Src/Bootloader -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Bootloader

clean-Core-2f-Src-2f-Bootloader:
	-$(RM) ./Core/Src/Bootloader/bootloader.d ./Core/Src/Bootloader/bootloader.o ./Core/Src/Bootloader/flash.d ./Core/Src/Bootloader/flash.o ./Core/Src/Bootloader/uart.d ./Core/Src/Bootloader/uart.o ./Core/Src/Bootloader/xmodem.d ./Core/Src/Bootloader/xmodem.o

.PHONY: clean-Core-2f-Src-2f-Bootloader

