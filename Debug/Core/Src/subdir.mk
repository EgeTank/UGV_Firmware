################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Odrive_can.c \
../Core/Src/blackbox.c \
../Core/Src/bms_emus_can.c \
../Core/Src/brake_control.c \
../Core/Src/can_comm.c \
../Core/Src/can_dma_buffer.c \
../Core/Src/current_limiter.c \
../Core/Src/drive_mode.c \
../Core/Src/encoder_reader.c \
../Core/Src/failsafe.c \
../Core/Src/fdcan.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/heartbeat.c \
../Core/Src/imu_filter.c \
../Core/Src/iwdg.c \
../Core/Src/main.c \
../Core/Src/mdma.c \
../Core/Src/motor_control.c \
../Core/Src/odrive_sm.c \
../Core/Src/pid.c \
../Core/Src/pid_tuner.c \
../Core/Src/protocol.c \
../Core/Src/safe_mode.c \
../Core/Src/safety.c \
../Core/Src/state_machine.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/unit_conversion.c \
../Core/Src/vel_profile.c \
../Core/Src/weapon_control.c 

OBJS += \
./Core/Src/Odrive_can.o \
./Core/Src/blackbox.o \
./Core/Src/bms_emus_can.o \
./Core/Src/brake_control.o \
./Core/Src/can_comm.o \
./Core/Src/can_dma_buffer.o \
./Core/Src/current_limiter.o \
./Core/Src/drive_mode.o \
./Core/Src/encoder_reader.o \
./Core/Src/failsafe.o \
./Core/Src/fdcan.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/heartbeat.o \
./Core/Src/imu_filter.o \
./Core/Src/iwdg.o \
./Core/Src/main.o \
./Core/Src/mdma.o \
./Core/Src/motor_control.o \
./Core/Src/odrive_sm.o \
./Core/Src/pid.o \
./Core/Src/pid_tuner.o \
./Core/Src/protocol.o \
./Core/Src/safe_mode.o \
./Core/Src/safety.o \
./Core/Src/state_machine.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/unit_conversion.o \
./Core/Src/vel_profile.o \
./Core/Src/weapon_control.o 

C_DEPS += \
./Core/Src/Odrive_can.d \
./Core/Src/blackbox.d \
./Core/Src/bms_emus_can.d \
./Core/Src/brake_control.d \
./Core/Src/can_comm.d \
./Core/Src/can_dma_buffer.d \
./Core/Src/current_limiter.d \
./Core/Src/drive_mode.d \
./Core/Src/encoder_reader.d \
./Core/Src/failsafe.d \
./Core/Src/fdcan.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/heartbeat.d \
./Core/Src/imu_filter.d \
./Core/Src/iwdg.d \
./Core/Src/main.d \
./Core/Src/mdma.d \
./Core/Src/motor_control.d \
./Core/Src/odrive_sm.d \
./Core/Src/pid.d \
./Core/Src/pid_tuner.d \
./Core/Src/protocol.d \
./Core/Src/safe_mode.d \
./Core/Src/safety.d \
./Core/Src/state_machine.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/unit_conversion.d \
./Core/Src/vel_profile.d \
./Core/Src/weapon_control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Odrive_can.cyclo ./Core/Src/Odrive_can.d ./Core/Src/Odrive_can.o ./Core/Src/Odrive_can.su ./Core/Src/blackbox.cyclo ./Core/Src/blackbox.d ./Core/Src/blackbox.o ./Core/Src/blackbox.su ./Core/Src/bms_emus_can.cyclo ./Core/Src/bms_emus_can.d ./Core/Src/bms_emus_can.o ./Core/Src/bms_emus_can.su ./Core/Src/brake_control.cyclo ./Core/Src/brake_control.d ./Core/Src/brake_control.o ./Core/Src/brake_control.su ./Core/Src/can_comm.cyclo ./Core/Src/can_comm.d ./Core/Src/can_comm.o ./Core/Src/can_comm.su ./Core/Src/can_dma_buffer.cyclo ./Core/Src/can_dma_buffer.d ./Core/Src/can_dma_buffer.o ./Core/Src/can_dma_buffer.su ./Core/Src/current_limiter.cyclo ./Core/Src/current_limiter.d ./Core/Src/current_limiter.o ./Core/Src/current_limiter.su ./Core/Src/drive_mode.cyclo ./Core/Src/drive_mode.d ./Core/Src/drive_mode.o ./Core/Src/drive_mode.su ./Core/Src/encoder_reader.cyclo ./Core/Src/encoder_reader.d ./Core/Src/encoder_reader.o ./Core/Src/encoder_reader.su ./Core/Src/failsafe.cyclo ./Core/Src/failsafe.d ./Core/Src/failsafe.o ./Core/Src/failsafe.su ./Core/Src/fdcan.cyclo ./Core/Src/fdcan.d ./Core/Src/fdcan.o ./Core/Src/fdcan.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/heartbeat.cyclo ./Core/Src/heartbeat.d ./Core/Src/heartbeat.o ./Core/Src/heartbeat.su ./Core/Src/imu_filter.cyclo ./Core/Src/imu_filter.d ./Core/Src/imu_filter.o ./Core/Src/imu_filter.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mdma.cyclo ./Core/Src/mdma.d ./Core/Src/mdma.o ./Core/Src/mdma.su ./Core/Src/motor_control.cyclo ./Core/Src/motor_control.d ./Core/Src/motor_control.o ./Core/Src/motor_control.su ./Core/Src/odrive_sm.cyclo ./Core/Src/odrive_sm.d ./Core/Src/odrive_sm.o ./Core/Src/odrive_sm.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pid_tuner.cyclo ./Core/Src/pid_tuner.d ./Core/Src/pid_tuner.o ./Core/Src/pid_tuner.su ./Core/Src/protocol.cyclo ./Core/Src/protocol.d ./Core/Src/protocol.o ./Core/Src/protocol.su ./Core/Src/safe_mode.cyclo ./Core/Src/safe_mode.d ./Core/Src/safe_mode.o ./Core/Src/safe_mode.su ./Core/Src/safety.cyclo ./Core/Src/safety.d ./Core/Src/safety.o ./Core/Src/safety.su ./Core/Src/state_machine.cyclo ./Core/Src/state_machine.d ./Core/Src/state_machine.o ./Core/Src/state_machine.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/unit_conversion.cyclo ./Core/Src/unit_conversion.d ./Core/Src/unit_conversion.o ./Core/Src/unit_conversion.su ./Core/Src/vel_profile.cyclo ./Core/Src/vel_profile.d ./Core/Src/vel_profile.o ./Core/Src/vel_profile.su ./Core/Src/weapon_control.cyclo ./Core/Src/weapon_control.d ./Core/Src/weapon_control.o ./Core/Src/weapon_control.su

.PHONY: clean-Core-2f-Src

