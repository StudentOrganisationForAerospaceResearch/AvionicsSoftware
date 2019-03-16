################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/AbortPhase.c \
../Src/EngineControl.c \
../Src/FlightPhase.c \
../Src/LogData.c \
../Src/MonitorForEmergencyShutoff.c \
../Src/ParachutesControl.c \
../Src/ReadAccelGyroMagnetism.c \
../Src/ReadBarometer.c \
../Src/ReadCombustionChamberPressure.c \
../Src/ReadGps.c \
../Src/ReadOxidizerTankPressure.c \
../Src/TransmitData.c \
../Src/Utils.c \
../Src/ValveControl.c \
../Src/freertos.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_TIM.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/AbortPhase.o \
./Src/EngineControl.o \
./Src/FlightPhase.o \
./Src/LogData.o \
./Src/MonitorForEmergencyShutoff.o \
./Src/ParachutesControl.o \
./Src/ReadAccelGyroMagnetism.o \
./Src/ReadBarometer.o \
./Src/ReadCombustionChamberPressure.o \
./Src/ReadGps.o \
./Src/ReadOxidizerTankPressure.o \
./Src/TransmitData.o \
./Src/Utils.o \
./Src/ValveControl.o \
./Src/freertos.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_TIM.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/AbortPhase.d \
./Src/EngineControl.d \
./Src/FlightPhase.d \
./Src/LogData.d \
./Src/MonitorForEmergencyShutoff.d \
./Src/ParachutesControl.d \
./Src/ReadAccelGyroMagnetism.d \
./Src/ReadBarometer.d \
./Src/ReadCombustionChamberPressure.d \
./Src/ReadGps.d \
./Src/ReadOxidizerTankPressure.d \
./Src/TransmitData.d \
./Src/Utils.d \
./Src/ValveControl.d \
./Src/freertos.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_TIM.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F405xx '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Inc" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Sudam.fernando/Desktop/AvionicsSoftware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


