################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/platform/src/vl53l1_platform.c 

OBJS += \
./Drivers/VL53L1X/platform/src/vl53l1_platform.o 

C_DEPS += \
./Drivers/VL53L1X/platform/src/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/platform/src/%.o Drivers/VL53L1X/platform/src/%.su: ../Drivers/VL53L1X/platform/src/%.c Drivers/VL53L1X/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/GitHub/CoMoBot/Drivers/VL53L1X/core/inc" -I"C:/GitHub/CoMoBot/Drivers/VL53L1X/core/src" -I"C:/GitHub/CoMoBot/Drivers/VL53L1X/platform/inc" -I"C:/GitHub/CoMoBot/Drivers/VL53L1X/platform/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-platform-2f-src

clean-Drivers-2f-VL53L1X-2f-platform-2f-src:
	-$(RM) ./Drivers/VL53L1X/platform/src/vl53l1_platform.d ./Drivers/VL53L1X/platform/src/vl53l1_platform.o ./Drivers/VL53L1X/platform/src/vl53l1_platform.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-platform-2f-src

