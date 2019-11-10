#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"
#include "Utils.h"
#include "FlightPhase.h"
#include "Data.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static const int TRANSMIT_DATA_PERIOD = 2000;

static const int8_t IMU_HEADER_BYTE = 0x31;
static const int8_t BAROMETER_HEADER_BYTE = 0x32;
static const int8_t GPS_HEADER_BYTE = 0x33;
static const int8_t OXIDIZER_TANK_HEADER_BYTE = 0x34;
static const int8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;
static const int8_t FLIGHT_PHASE_HEADER_BYTE = 0x36;
static const int8_t INJECTION_VALVE_STATUS_HEADER_BYTE = 0x38;
static const int8_t LOWER_VALVE_STATUS_HEADER_BYTE = 0x39;

#define IMU_SERIAL_MSG_SIZE (41)
#define BAROMETER_SERIAL_MSG_SIZE (13)
#define GPS_SERIAL_MSG_SIZE (48)
#define OXIDIZER_TANK_SERIAL_MSG_SIZE (9)
#define COMBUSTION_CHAMBER_SERIAL_MSG_SIZE (9)
#define FLIGHT_PHASE_SERIAL_MSG_SIZE (6)

static const uint8_t UART_TIMEOUT = 100;

void transmitImuData(AllData* data)
{
    int32_t accelX = -1;
    int32_t accelY = -1;
    int32_t accelZ = -1;
    int32_t gyroX = -1;
    int32_t gyroY = -1;
    int32_t gyroZ = -1;
    int32_t magnetoX = -1;
    int32_t magnetoY = -1;
    int32_t magnetoZ = -1;

    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK)
    {
        accelX = data->accelGyroMagnetismData_->accelX_;
        accelY = data->accelGyroMagnetismData_->accelY_;
        accelZ = data->accelGyroMagnetismData_->accelZ_;
        gyroX = data->accelGyroMagnetismData_->gyroX_;
        gyroY = data->accelGyroMagnetismData_->gyroY_;
        gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(data->accelGyroMagnetismData_->mutex_);
    }

    uint8_t buffer[IMU_SERIAL_MSG_SIZE] = {0};

    buffer[0] = IMU_HEADER_BYTE;
    buffer[1] = IMU_HEADER_BYTE;
    buffer[2] = IMU_HEADER_BYTE;
    buffer[3] = IMU_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, accelX);
    writeInt32ToArray(&buffer, 8, accelY);
    writeInt32ToArray(&buffer, 12, accelZ);
    writeInt32ToArray(&buffer, 16, gyroX);
    writeInt32ToArray(&buffer, 20, gyroY);
    writeInt32ToArray(&buffer, 24, gyroZ);
    writeInt32ToArray(&buffer, 28, magnetoX);
    writeInt32ToArray(&buffer, 32, magnetoY);
    writeInt32ToArray(&buffer, 36, magnetoZ);
    buffer[IMU_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitBarometerData(AllData* data)
{
    int32_t pressure = -1;
    int32_t temperature = -1;

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        pressure = data->barometerData_->pressure_;
        temperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    uint8_t buffer[BAROMETER_SERIAL_MSG_SIZE] = {0};

    buffer[0] = BAROMETER_HEADER_BYTE;
    buffer[1] = BAROMETER_HEADER_BYTE;
    buffer[2] = BAROMETER_HEADER_BYTE;
    buffer[3] = BAROMETER_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, pressure);
    writeInt32ToArray(&buffer, 8, temperature);
    buffer[BAROMETER_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);	// Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitGpsData(AllData* data)
{
	uint32_t time;

	uint32_t latitude_degrees;
	uint32_t latitude_minutes;
    char latitude_direction;
    int32_t latitude_degrees_with_direction;

	uint32_t longitude_degrees;
	uint32_t longitude_minutes;
    char longitude_direction;
    int32_t longitude_degrees_with_direction;

    int32_t antennaAltitude_altitude;
    char antennaAltitude_unit;

    int32_t geoidAltitude_altitude;
    char geoidAltitude_unit;

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
    	time = data->gpsData_->time_;

    	latitude_degrees = data->gpsData_->latitude_.degrees_;
    	latitude_minutes = data->gpsData_->latitude_.minutes_;
        latitude_direction = data->gpsData_->latitude_.direction_;
        latitude_degrees_with_direction = data->gpsData_->latitude_.degrees_with_direction_;

    	longitude_degrees = data->gpsData_->longitude_.degrees_;
    	longitude_minutes = data->gpsData_->longitude_.minutes_;
        longitude_direction = data->gpsData_->longitude_.direction_;
        longitude_degrees_with_direction = data->gpsData_->longitude_.degrees_with_direction_;

        antennaAltitude_altitude = data->gpsData_->antennaAltitude_.altitude_;
        antennaAltitude_unit = data->gpsData_->antennaAltitude_.unit_;

        geoidAltitude_altitude = data->gpsData_->geoidAltitude_.altitude_;
        geoidAltitude_unit = data->gpsData_->geoidAltitude_.unit_;

        osMutexRelease(data->gpsData_->mutex_);
    }

    uint8_t buffer[GPS_SERIAL_MSG_SIZE] = {0};

   buffer[0] = GPS_HEADER_BYTE;
   buffer[1] = GPS_HEADER_BYTE;
   buffer[2] = GPS_HEADER_BYTE;
   buffer[3] = GPS_HEADER_BYTE;
   // Does this need to be uint32toarray???
   writeInt32ToArray(&buffer, 4, time);

   writeInt32ToArray(&buffer, 8, latitude_degrees);
   writeInt32ToArray(&buffer, 12, latitude_minutes);
   buffer[16] = latitude_direction;
   writeInt32ToArray(&buffer, 17, latitude_degrees_with_direction);

   writeInt32ToArray(&buffer, 21, longitude_degrees);
   writeInt32ToArray(&buffer, 25, longitude_minutes);
   buffer[29] = latitude_direction;
   writeInt32ToArray(&buffer, 30, longitude_degrees_with_direction);

   writeInt32ToArray(&buffer, 34, antennaAltitude_altitude);
   buffer[38] = antennaAltitude_unit;

   writeInt32ToArray(&buffer, 42, geoidAltitude_altitude);
   buffer[46] = geoidAltitude_unit;

   buffer[GPS_SERIAL_MSG_SIZE - 1] = 0x00;

   if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
   {
       HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
   }

   HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitOxidizerTankData(AllData* data)
{
    int32_t oxidizerTankPressure = -1;

    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    uint8_t buffer[OXIDIZER_TANK_SERIAL_MSG_SIZE] = {0};

    buffer[0] = OXIDIZER_TANK_HEADER_BYTE;
    buffer[1] = OXIDIZER_TANK_HEADER_BYTE;
    buffer[2] = OXIDIZER_TANK_HEADER_BYTE;
    buffer[3] = OXIDIZER_TANK_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, oxidizerTankPressure);
    buffer[OXIDIZER_TANK_SERIAL_MSG_SIZE - 1] = 0x00;


    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitCombustionChamberData(AllData* data)
{
    int32_t combustionChamberPressure = -1;

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

    uint8_t buffer[COMBUSTION_CHAMBER_SERIAL_MSG_SIZE] = {0};

    buffer[0] = COMBUSTION_CHAMBER_HEADER_BYTE;
    buffer[1] = COMBUSTION_CHAMBER_HEADER_BYTE;
    buffer[2] = COMBUSTION_CHAMBER_HEADER_BYTE;
    buffer[3] = COMBUSTION_CHAMBER_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, combustionChamberPressure);
    buffer[COMBUSTION_CHAMBER_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitFlightPhaseData(AllData* data)
{
    uint8_t flightPhase = getCurrentFlightPhase();

    uint8_t buffer [] = {FLIGHT_PHASE_HEADER_BYTE,
                         FLIGHT_PHASE_HEADER_BYTE,
                         FLIGHT_PHASE_HEADER_BYTE,
                         FLIGHT_PHASE_HEADER_BYTE,
                         flightPhase,
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitInjectionValveStatus()
{
    uint8_t injectionValveStatus = injectionValveIsOpen;

    uint8_t buffer [] = {INJECTION_VALVE_STATUS_HEADER_BYTE,
                         INJECTION_VALVE_STATUS_HEADER_BYTE,
                         INJECTION_VALVE_STATUS_HEADER_BYTE,
                         INJECTION_VALVE_STATUS_HEADER_BYTE,
                         (uint8_t) ((injectionValveStatus)),
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
}

void transmitLowerVentValveStatus()
{
    uint8_t ventValveStatus = lowerVentValveIsOpen;

    uint8_t buffer [] = {LOWER_VALVE_STATUS_HEADER_BYTE,
                         LOWER_VALVE_STATUS_HEADER_BYTE,
                         LOWER_VALVE_STATUS_HEADER_BYTE,
                         LOWER_VALVE_STATUS_HEADER_BYTE,
                         (uint8_t) ((ventValveStatus)),
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
}

void transmitDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, TRANSMIT_DATA_PERIOD);

        transmitImuData(data);
        transmitBarometerData(data);
        transmitGpsData(data);
        transmitOxidizerTankData(data);
        transmitCombustionChamberData(data);
        transmitFlightPhaseData(data);
        transmitInjectionValveStatus();
        transmitLowerVentValveStatus();
        HAL_UART_Receive_IT(&huart2, &launchSystemsRxChar, 1);
    }
}
