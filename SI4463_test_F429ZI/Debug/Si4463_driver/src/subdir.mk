################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Si4463_driver/src/ax25_huang.c \
../Si4463_driver/src/si4463_huang.c 

OBJS += \
./Si4463_driver/src/ax25_huang.o \
./Si4463_driver/src/si4463_huang.o 

C_DEPS += \
./Si4463_driver/src/ax25_huang.d \
./Si4463_driver/src/si4463_huang.d 


# Each subdirectory must supply rules for building sources it contributes
Si4463_driver/src/%.o Si4463_driver/src/%.su Si4463_driver/src/%.cyclo: ../Si4463_driver/src/%.c Si4463_driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/e1406/Documents/STM32CubeIDE/workspace_1.11.2/SI4463_test_F429ZI/Si4463_driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Si4463_driver-2f-src

clean-Si4463_driver-2f-src:
	-$(RM) ./Si4463_driver/src/ax25_huang.cyclo ./Si4463_driver/src/ax25_huang.d ./Si4463_driver/src/ax25_huang.o ./Si4463_driver/src/ax25_huang.su ./Si4463_driver/src/si4463_huang.cyclo ./Si4463_driver/src/si4463_huang.d ./Si4463_driver/src/si4463_huang.o ./Si4463_driver/src/si4463_huang.su

.PHONY: clean-Si4463_driver-2f-src

