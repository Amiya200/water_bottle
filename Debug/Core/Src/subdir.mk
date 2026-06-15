################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_logic.c \
../Core/Src/battery.c \
../Core/Src/ble_jdy29.c \
../Core/Src/ble_protocol.c \
../Core/Src/bma253.c \
../Core/Src/bringup_test.c \
../Core/Src/button_test.c \
../Core/Src/buzzer.c \
../Core/Src/data_storage.c \
../Core/Src/device_state.c \
../Core/Src/ee_data.c \
../Core/Src/ee_store.c \
../Core/Src/hx711.c \
../Core/Src/main.c \
../Core/Src/ntc_temp.c \
../Core/Src/rtc_manager.c \
../Core/Src/rtc_pcf8563.c \
../Core/Src/sensors.c \
../Core/Src/stm32f0xx_hal_msp.c \
../Core/Src/stm32f0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f0xx.c \
../Core/Src/tds_sensor.c \
../Core/Src/ws2812b.c 

OBJS += \
./Core/Src/app_logic.o \
./Core/Src/battery.o \
./Core/Src/ble_jdy29.o \
./Core/Src/ble_protocol.o \
./Core/Src/bma253.o \
./Core/Src/bringup_test.o \
./Core/Src/button_test.o \
./Core/Src/buzzer.o \
./Core/Src/data_storage.o \
./Core/Src/device_state.o \
./Core/Src/ee_data.o \
./Core/Src/ee_store.o \
./Core/Src/hx711.o \
./Core/Src/main.o \
./Core/Src/ntc_temp.o \
./Core/Src/rtc_manager.o \
./Core/Src/rtc_pcf8563.o \
./Core/Src/sensors.o \
./Core/Src/stm32f0xx_hal_msp.o \
./Core/Src/stm32f0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f0xx.o \
./Core/Src/tds_sensor.o \
./Core/Src/ws2812b.o 

C_DEPS += \
./Core/Src/app_logic.d \
./Core/Src/battery.d \
./Core/Src/ble_jdy29.d \
./Core/Src/ble_protocol.d \
./Core/Src/bma253.d \
./Core/Src/bringup_test.d \
./Core/Src/button_test.d \
./Core/Src/buzzer.d \
./Core/Src/data_storage.d \
./Core/Src/device_state.d \
./Core/Src/ee_data.d \
./Core/Src/ee_store.d \
./Core/Src/hx711.d \
./Core/Src/main.d \
./Core/Src/ntc_temp.d \
./Core/Src/rtc_manager.d \
./Core/Src/rtc_pcf8563.d \
./Core/Src/sensors.d \
./Core/Src/stm32f0xx_hal_msp.d \
./Core/Src/stm32f0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f0xx.d \
./Core/Src/tds_sensor.d \
./Core/Src/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x6 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -flto -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_logic.cyclo ./Core/Src/app_logic.d ./Core/Src/app_logic.o ./Core/Src/app_logic.su ./Core/Src/battery.cyclo ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/ble_jdy29.cyclo ./Core/Src/ble_jdy29.d ./Core/Src/ble_jdy29.o ./Core/Src/ble_jdy29.su ./Core/Src/ble_protocol.cyclo ./Core/Src/ble_protocol.d ./Core/Src/ble_protocol.o ./Core/Src/ble_protocol.su ./Core/Src/bma253.cyclo ./Core/Src/bma253.d ./Core/Src/bma253.o ./Core/Src/bma253.su ./Core/Src/bringup_test.cyclo ./Core/Src/bringup_test.d ./Core/Src/bringup_test.o ./Core/Src/bringup_test.su ./Core/Src/button_test.cyclo ./Core/Src/button_test.d ./Core/Src/button_test.o ./Core/Src/button_test.su ./Core/Src/buzzer.cyclo ./Core/Src/buzzer.d ./Core/Src/buzzer.o ./Core/Src/buzzer.su ./Core/Src/data_storage.cyclo ./Core/Src/data_storage.d ./Core/Src/data_storage.o ./Core/Src/data_storage.su ./Core/Src/device_state.cyclo ./Core/Src/device_state.d ./Core/Src/device_state.o ./Core/Src/device_state.su ./Core/Src/ee_data.cyclo ./Core/Src/ee_data.d ./Core/Src/ee_data.o ./Core/Src/ee_data.su ./Core/Src/ee_store.cyclo ./Core/Src/ee_store.d ./Core/Src/ee_store.o ./Core/Src/ee_store.su ./Core/Src/hx711.cyclo ./Core/Src/hx711.d ./Core/Src/hx711.o ./Core/Src/hx711.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ntc_temp.cyclo ./Core/Src/ntc_temp.d ./Core/Src/ntc_temp.o ./Core/Src/ntc_temp.su ./Core/Src/rtc_manager.cyclo ./Core/Src/rtc_manager.d ./Core/Src/rtc_manager.o ./Core/Src/rtc_manager.su ./Core/Src/rtc_pcf8563.cyclo ./Core/Src/rtc_pcf8563.d ./Core/Src/rtc_pcf8563.o ./Core/Src/rtc_pcf8563.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/stm32f0xx_hal_msp.cyclo ./Core/Src/stm32f0xx_hal_msp.d ./Core/Src/stm32f0xx_hal_msp.o ./Core/Src/stm32f0xx_hal_msp.su ./Core/Src/stm32f0xx_it.cyclo ./Core/Src/stm32f0xx_it.d ./Core/Src/stm32f0xx_it.o ./Core/Src/stm32f0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f0xx.cyclo ./Core/Src/system_stm32f0xx.d ./Core/Src/system_stm32f0xx.o ./Core/Src/system_stm32f0xx.su ./Core/Src/tds_sensor.cyclo ./Core/Src/tds_sensor.d ./Core/Src/tds_sensor.o ./Core/Src/tds_sensor.su ./Core/Src/ws2812b.cyclo ./Core/Src/ws2812b.d ./Core/Src/ws2812b.o ./Core/Src/ws2812b.su

.PHONY: clean-Core-2f-Src

