#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"
#include "Utils.h"
#include "FlightPhase.h"
#include "Data.h"

static const int TRANSMIT_DATA_PERIOD = 500;

static const int8_t IMU_HEADER_BYTE = 0x31;
static const int8_t BAROMETER_HEADER_BYTE = 0x32;
static const int8_t GPS_HEADER_BYTE = 0x33;
static const int8_t OXIDIZER_TANK_HEADER_BYTE = 0x34;
static const int8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;
static const int8_t FLIGHT_PHASE_HEADER_BYTE = 0x36;
static const int8_t VENT_VALVE_STATUS_HEADER_BYTE = 0x37;

#define IMU_SERIAL_MSG_SIZE (41)
#define BAROMETER_SERIAL_MSG_SIZE (13)
#define GPS_SERIAL_MSG_SIZE (21)
#define OXIDIZER_TANK_SERIAL_MSG_SIZE (9)
#define COMBUSTION_CHAMBER_SERIAL_MSG_SIZE (9)
#define FLIGHT_PHASE_SERIAL_MSG_SIZE (6)
#define VENT_VALVE_STATUS_SERIAL_MSG_SIZE (6)

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

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
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

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);	// Launch Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitGpsData(AllData* data)
{
    int32_t altitude = -1;
    int32_t epochTimeMsec = -1;
    int32_t latitude = -1;
    int32_t longitude = -1;

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        altitude = data->gpsData_->altitude_;
        epochTimeMsec = data->gpsData_->epochTimeMsec_;
        latitude = data->gpsData_->latitude_;
        longitude = data->gpsData_->longitude_;
        osMutexRelease(data->gpsData_->mutex_);
    }

    uint8_t buffer[GPS_SERIAL_MSG_SIZE] = {0};

    buffer[0] = GPS_HEADER_BYTE;
    buffer[1] = GPS_HEADER_BYTE;
    buffer[2] = GPS_HEADER_BYTE;
    buffer[3] = GPS_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, altitude);
    writeInt32ToArray(&buffer, 8, epochTimeMsec);
    writeInt32ToArray(&buffer, 12, latitude);
    writeInt32ToArray(&buffer, 16, longitude);
    buffer[GPS_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Launch Systems
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


    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
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

    if (getCurrentFlightPhase() == PRELAUNCH)
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
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

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitVentValveStatus()
{
    uint8_t ventValveStatus = ventValveIsOpen;

    uint8_t buffer [] = {VENT_VALVE_STATUS_HEADER_BYTE,
                         VENT_VALVE_STATUS_HEADER_BYTE,
                         VENT_VALVE_STATUS_HEADER_BYTE,
                         VENT_VALVE_STATUS_HEADER_BYTE,
                         (uint8_t) ((ventValveStatus)),
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Launch Systems
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
        transmitVentValveStatus();
        HAL_UART_Receive_IT(&huart2, &launchSystemsRxChar, 1);
    }
}