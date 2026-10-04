################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/esc.c \
../Core/Src/sbus.c \
../Core/Src/pi_link.c \
../Core/Src/failsafe.c \
../Core/Src/battery.c \
../Core/Src/imu.c \
../Core/Src/settings.c \
../Core/Src/wdg.c \
../Core/Src/mode_switch.c \
../Core/Src/arming.c \
../Core/Src/motor.c \
../Core/Src/clock.c \
../Core/Src/clock_core.c \
../Core/Src/imu_reports.c

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/esc.o \
./Core/Src/sbus.o \
./Core/Src/pi_link.o \
./Core/Src/failsafe.o \
./Core/Src/battery.o \
./Core/Src/imu.o \
./Core/Src/settings.o \
./Core/Src/wdg.o \
./Core/Src/mode_switch.o \
./Core/Src/arming.o \
./Core/Src/motor.o \
./Core/Src/clock.o \
./Core/Src/clock_core.o \
./Core/Src/imu_reports.o

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/esc.d \
./Core/Src/sbus.d \
./Core/Src/pi_link.d \
./Core/Src/failsafe.d \
./Core/Src/battery.d \
./Core/Src/imu.d \
./Core/Src/settings.d \
./Core/Src/wdg.d \
./Core/Src/mode_switch.d \
./Core/Src/arming.d \
./Core/Src/motor.d \
./Core/Src/clock.d \
./Core/Src/clock_core.d \
./Core/Src/imu_reports.d


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

