################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/platform/vl53l1_platform.c 

OBJS += \
./Drivers/VL53L1X/platform/vl53l1_platform.o 

C_DEPS += \
./Drivers/VL53L1X/platform/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/platform/%.o Drivers/VL53L1X/platform/%.su: ../Drivers/VL53L1X/platform/%.c Drivers/VL53L1X/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/GitHub/CoMoBot/motor/Drivers/VL53L1X/core" -I"C:/GitHub/CoMoBot/motor/Drivers/VL53L1X/platform" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-platform

clean-Drivers-2f-VL53L1X-2f-platform:
	-$(RM) ./Drivers/VL53L1X/platform/vl53l1_platform.d ./Drivers/VL53L1X/platform/vl53l1_platform.o ./Drivers/VL53L1X/platform/vl53l1_platform.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-platform

