################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BNOStatemachine/BNOStatemachine.c 

OBJS += \
./BNOStatemachine/BNOStatemachine.o 

C_DEPS += \
./BNOStatemachine/BNOStatemachine.d 


# Each subdirectory must supply rules for building sources it contributes
BNOStatemachine/%.o BNOStatemachine/%.su BNOStatemachine/%.cyclo: ../BNOStatemachine/%.c BNOStatemachine/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Oled -I../LKF -I../CAN_Handle -I../BNO055 -I../BNOStatemachine -I../CANlibrary -I../CRC -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BNOStatemachine

clean-BNOStatemachine:
	-$(RM) ./BNOStatemachine/BNOStatemachine.cyclo ./BNOStatemachine/BNOStatemachine.d ./BNOStatemachine/BNOStatemachine.o ./BNOStatemachine/BNOStatemachine.su

.PHONY: clean-BNOStatemachine

