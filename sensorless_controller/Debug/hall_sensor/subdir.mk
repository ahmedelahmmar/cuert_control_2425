################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hall_sensor/hall_sensor.c 

OBJS += \
./hall_sensor/hall_sensor.o 

C_DEPS += \
./hall_sensor/hall_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
hall_sensor/%.o hall_sensor/%.su hall_sensor/%.cyclo: ../hall_sensor/%.c hall_sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-hall_sensor

clean-hall_sensor:
	-$(RM) ./hall_sensor/hall_sensor.cyclo ./hall_sensor/hall_sensor.d ./hall_sensor/hall_sensor.o ./hall_sensor/hall_sensor.su

.PHONY: clean-hall_sensor

