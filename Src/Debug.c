/**
  ******************************************************************************
  * File Name          : Debug.c
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "Debug.h"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/
typedef struct{
    int32_t accelX;
    int32_t accelY;
    int32_t accelZ;
    int32_t gyroX;
    int32_t gyroY;
    int32_t gyroZ;
    int32_t magnetoX;
    int32_t magnetoY;
    int32_t magnetoZ;
    int32_t barometerPressure;
    int32_t barometerTemperature;
    int32_t combustionChamberPressure;
    int32_t oxidizerTankPressure;
    int32_t gps_time;
    int32_t latitude_degrees;
    int32_t latitude_minutes;
    int32_t longitude_degrees;
    int32_t longitude_minutes;
    int32_t antennaAltitude;
    int32_t geoidAltitude;
    int32_t altitude;
    int32_t currentFlightPhase;
    int32_t tick;
} allData;
/* Constants -----------------------------------------------------------------*/
static const int DEBUG_TASK_PERIOD = 100;
static const uint16_t memAddress;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

// TODO: Only run thread when appropriate GPIO pin pulled HIGH
void debugTask(void const* arg) {
    uint32_t prevWakeTime = osKernelSysTick();
    uint8_t buffer = 0x00;
    allData* debugData;

	while (1) {
		osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);

		HAL_UART_Receive(&huart5, &buffer, 1, 1000);

		// LOGIC
		switch(buffer){
			case 't':
				initializeLogEntry(allData);
				writeLogEntryToEEPROM(memAddress,allData);
				// TODO: transmit allData struct through UART
				break;
			case 'd':
				readLogEntryFromEEPROM(memAddress,allData);
				// TODO: transmit allDara struct through UART
				break;
			default:
		}
	}
}
