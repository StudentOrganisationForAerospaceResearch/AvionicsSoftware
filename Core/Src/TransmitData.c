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


CRC_HandleTypeDef hcrc;
static const int TRANSMIT_DATA_PERIOD = 1000;

static const int8_t IMU_HEADER_BYTE = 0x10;
static const int8_t BAROMETER_HEADER_BYTE = 0x11;
static const int8_t GPS_HEADER_BYTE = 0x12;
static const int8_t OXIDIZER_TANK_HEADER_BYTE = 0x34; 			// not used
static const int8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;		// not used
static const int8_t FLIGHT_PHASE_HEADER_BYTE = 0x13;
static const int8_t INJECTION_VALVE_STATUS_HEADER_BYTE = 0x38; 	// not used
static const int8_t LOWER_VALVE_STATUS_HEADER_BYTE = 0x39;		// not used
static const int8_t BATTERY_VOLTAGE_HEADER_BYTE = 0x14;
static const int8_t TICK_HEADER_BYTE = 0x15;
static const int8_t CRC_HEADER_BYTE = 0x16;						// needed?

#define F0_ESCAPE (0xF0)
#define START_FLAG (0x00)
#define END_FLAG (0X01)

static const uint8_t UART_TIMEOUT = 100;

void transmitLogEntry(LogEntry* givenLog)
{
	int16_t bufferIndex = 0;
    uint8_t* transmitBuffer = pvPortMalloc( sizeof(LogEntry)*3 );

    // start flag	0xF000
	transmitBuffer[bufferIndex++] = F0_ESCAPE;
    transmitBuffer[bufferIndex++] = START_FLAG;

    // IMU flag 	0xF010
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = IMU_HEADER_BYTE;
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->accelX);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->accelY);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->accelZ);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->gyroX);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->gyroY);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->gyroZ);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->magnetoX);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->magnetoY);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->magnetoZ);

    // Baro flag 	0xF011
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = BAROMETER_HEADER_BYTE;
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->barometerPressure);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->barometerTemperature);

    // GPS flag 	0xF012
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = GPS_HEADER_BYTE;
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->gps_time);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->latitude_degrees);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->latitude_minutes);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->longitude_degrees);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->longitude_minutes);
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->altitude);

    // flight phase flag 	0xF013
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = FLIGHT_PHASE_HEADER_BYTE;
    bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->currentFlightPhase);

    // battery voltage flag 	0xF014
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
    transmitBuffer[bufferIndex++] = BATTERY_VOLTAGE_HEADER_BYTE;
    bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->batteryVoltage);

    // tick flag 	0xF015
	transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = TICK_HEADER_BYTE;
	bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, givenLog->tick);

	// CRC flag		0xF016
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = CRC_HEADER_BYTE;
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)transmitBuffer, bufferIndex - 1);
    bufferIndex = writeInt32ToArrayEncoded(transmitBuffer, bufferIndex, crc);

	// END flag		0xF001
    transmitBuffer[bufferIndex++] = F0_ESCAPE;
	transmitBuffer[bufferIndex++] = END_FLAG;

	HAL_UART_Transmit(&huart2, transmitBuffer, bufferIndex, UART_TIMEOUT);

    vPortFree(transmitBuffer);
}

void transmitDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    LogEntry log;
	initializeLogEntry(&log);

    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, TRANSMIT_DATA_PERIOD);
        updateLogEntry(data, &log);
        transmitLogEntry(&log);
        HAL_UART_Receive_IT(&huart2, &launchSystemsRxChar, 1);
    }
}

