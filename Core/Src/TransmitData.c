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


static const int TRANSMIT_DATA_PERIOD = 1000;

static const int8_t IMU_HEADER_BYTE = 0x31;
static const int8_t BAROMETER_HEADER_BYTE = 0x32;
static const int8_t GPS_HEADER_BYTE = 0x33;
static const int8_t OXIDIZER_TANK_HEADER_BYTE = 0x34;
static const int8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;
static const int8_t FLIGHT_PHASE_HEADER_BYTE = 0x36;
static const int8_t INJECTION_VALVE_STATUS_HEADER_BYTE = 0x38;
static const int8_t LOWER_VALVE_STATUS_HEADER_BYTE = 0x39;

#define START_FLAG (0xF0)
#define END_FLAG (0XF0)
#define F0_ESCAPE (0xF0)
#define F0_REPLACEMENT_1 (0xF1)
#define F0_REPLACEMENT_2 (0xF2)
#define F1_ESCAPE (0xF1)
#define F1_REPLACEMENT_1 (0xF1)
#define F1_REPLACEMENT_2 (0xF3)
#define FLAGS_AND_CRC_SIZE (6) //1 byte for header flag, 1 byte for ender flag, 4 bytes for crc
#define IMU_SERIAL_MSG_SIZE (37)
#define BAROMETER_SERIAL_MSG_SIZE (9)
#define GPS_SERIAL_MSG_SIZE (25)
#define OXIDIZER_TANK_SERIAL_MSG_SIZE (5)
#define COMBUSTION_CHAMBER_SERIAL_MSG_SIZE (5)
#define ONE_BYTE_SERIAL_MSG_SIZE (2)

static const uint8_t UART_TIMEOUT = 100;

//Encodes a message composed of data with a header and ender flag, uses byte stuffing to replace instances of overlaps.
//Finally generates and adds a 32 bit crc at the end based on the buffer (**not the message, but the buffer with flags).
void encodeMessage(uint8_t* message, int messageLength, uint8_t* buffer)
{
    int bufferIndex = 1;
    buffer[0] = START_FLAG;

    for (int i = 0; i < messageLength; i++)
    {
        //If the byte is F0, replace with F1F2
        if (message[i] == F0_ESCAPE)
        {
            buffer[bufferIndex++] = F0_REPLACEMENT_1;
            buffer[bufferIndex++] = F0_REPLACEMENT_2;
        }
        //If the byte is F1, replace with F1F3
        else if (message[i] == F1_ESCAPE)
        {
            buffer[bufferIndex++] = F1_REPLACEMENT_1;
            buffer[bufferIndex++] = F1_REPLACEMENT_2;
        }
        else
        {
            buffer[bufferIndex++] = message[i];
        }
    }

    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)message, messageLength - 1);
    writeInt32ToArray(buffer, bufferIndex, crc);
    bufferIndex += 4;
    buffer[bufferIndex] = END_FLAG;
}

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

    //obtain current values from sensors
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

    //construct the message in the format accelXYZ, gyroXYZ, MagnetoXYZ
    uint8_t message[IMU_SERIAL_MSG_SIZE] = { 0 };
    int messageIndex = 0;
    message[0] = IMU_HEADER_BYTE;
    messageIndex++;
    writeInt32ToArray(message, messageIndex, accelX);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, accelY);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, accelZ);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, gyroX);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, gyroY);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, gyroZ);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, magnetoX);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, magnetoY);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, magnetoZ);
    messageIndex += 4;
    //Find the final length of the encoded buffer based on number of overlaps
    int encodedMessageLength = IMU_SERIAL_MSG_SIZE;

    for (int i = 0; i < IMU_SERIAL_MSG_SIZE; i++)
    {
        //If byte is F0 or F1, requires two bytes to represent(F1F2 or F1F3), so one additional byte is needed every time F0 or F1 is encountered
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    //Dynamic allocate a buffer because variable length arrays are not allowed
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    //Encode the message and send it to ground systems and radio
    encodeMessage(message, IMU_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
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

    uint8_t message[BAROMETER_SERIAL_MSG_SIZE] = { 0 };
    int messageIndex = 0;
    message[0] = BAROMETER_HEADER_BYTE;
    messageIndex++;
    writeInt32ToArray(message, messageIndex, pressure);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, temperature);
    messageIndex += 4;
    int encodedMessageLength = BAROMETER_SERIAL_MSG_SIZE;

    for (int i = 0; i < BAROMETER_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, BAROMETER_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
}

void transmitGpsData(AllData* data)
{
    uint32_t gps_time = 0xFFFF;
    int32_t latitude_degrees = -1;
    int32_t latitude_minutes = -1;
    int32_t longitude_degrees = -1;
    int32_t longitude_minutes = -1;
    int32_t altitude = -1;

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        gps_time = data->gpsData_->time_;

        latitude_degrees = data->gpsData_->latitude_.degrees_;
        latitude_minutes = data->gpsData_->latitude_.minutes_;

        longitude_degrees = data->gpsData_->longitude_.degrees_;
        longitude_minutes = data->gpsData_->longitude_.minutes_;

        altitude = data->gpsData_->totalAltitude_.altitude_;

        osMutexRelease(data->gpsData_->mutex_);
    }

    uint8_t message[GPS_SERIAL_MSG_SIZE] = { 0 };
    int messageIndex = 0;
    message[0] = GPS_HEADER_BYTE;
    messageIndex++;
    writeInt32ToArray(message, messageIndex, gps_time);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, latitude_degrees);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, latitude_minutes);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, longitude_degrees);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, longitude_minutes);
    messageIndex += 4;
    writeInt32ToArray(message, messageIndex, altitude);
    messageIndex += 4;

    int encodedMessageLength = GPS_SERIAL_MSG_SIZE;

    for (int i = 0; i < GPS_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, GPS_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
}

void transmitOxidizerTankData(AllData* data)
{
    int32_t oxidizerTankPressure = -1;

    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    uint8_t message[OXIDIZER_TANK_SERIAL_MSG_SIZE] = { 0 };
    int messageIndex = 0;
    message[0] = OXIDIZER_TANK_HEADER_BYTE;
    messageIndex++;
    writeInt32ToArray(message, messageIndex, oxidizerTankPressure);
    messageIndex += 4;

    int encodedMessageLength = OXIDIZER_TANK_SERIAL_MSG_SIZE;

    for (int i = 0; i < OXIDIZER_TANK_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, OXIDIZER_TANK_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
}

void transmitCombustionChamberData(AllData* data)
{
    int32_t combustionChamberPressure = -1;

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

    uint8_t message[COMBUSTION_CHAMBER_SERIAL_MSG_SIZE] = { 0 };
    int messageIndex = 0;
    message[0] = COMBUSTION_CHAMBER_HEADER_BYTE;
    messageIndex++;
    writeInt32ToArray(message, messageIndex, combustionChamberPressure);
    messageIndex += 4;

    int encodedMessageLength = COMBUSTION_CHAMBER_SERIAL_MSG_SIZE;

    for (int i = 0; i < COMBUSTION_CHAMBER_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, COMBUSTION_CHAMBER_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
}

void transmitFlightPhaseData(AllData* data)
{
    uint8_t flightPhase = getCurrentFlightPhase();

    uint8_t message [] = {FLIGHT_PHASE_HEADER_BYTE,
                          flightPhase,
                         };

    int encodedMessageLength = ONE_BYTE_SERIAL_MSG_SIZE;

    for (int i = 0; i < ONE_BYTE_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, ONE_BYTE_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
}

void transmitInjectionValveStatus()
{
    uint8_t injectionValveStatus = injectionValveIsOpen;

    uint8_t message [] = {INJECTION_VALVE_STATUS_HEADER_BYTE,
                          (uint8_t) ((injectionValveStatus)),
                         };
    int encodedMessageLength = ONE_BYTE_SERIAL_MSG_SIZE;

    for (int i = 0; i < ONE_BYTE_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, ONE_BYTE_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
}

void transmitLowerVentValveStatus()
{
    uint8_t ventValveStatus = lowerVentValveIsOpen;

    uint8_t message [] = {LOWER_VALVE_STATUS_HEADER_BYTE,
                          (uint8_t) ((ventValveStatus)),
                         };
    int encodedMessageLength = ONE_BYTE_SERIAL_MSG_SIZE;

    for (int i = 0; i < ONE_BYTE_SERIAL_MSG_SIZE; i++)
    {
        if (message[i] == F0_ESCAPE || message[i] == F1_ESCAPE)
        {
            encodedMessageLength++;
        }
    }

    int bufferLength = encodedMessageLength + FLAGS_AND_CRC_SIZE;
    uint8_t* buffer = malloc(bufferLength * sizeof(uint8_t));
    encodeMessage(message, ONE_BYTE_SERIAL_MSG_SIZE, buffer);

    HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
    free(buffer);
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

