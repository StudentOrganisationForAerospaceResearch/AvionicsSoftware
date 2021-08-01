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

/* Constants -----------------------------------------------------------------*/
static const int DEBUG_TASK_PERIOD = 100;
static const uint16_t memAddress = 0x07; 

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

// TODO: Only run thread when appropriate GPIO pin pulled HIGH
void debugTask(void const* arg) {
    uint32_t prevWakeTime = osKernelSysTick();
    uint8_t buffer = 0x00;
    LogEntry* debugData;

	while (1) {
		osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);
		HAL_UART_Receive(&huart5, &buffer, 1, 1000);

		// LOGIC
		switch(buffer){
			case 't':
				initializeLogEntry(debugData);
				writeLogEntryToEEPROM(memAddress,debugData);
				HAL_UART_TRANSMIT(&huart5,debugData,sizeof(*debugData),1000);
				break;
			case 'd':
				readLogEntryFromEEPROM(memAddress,debugData);
				HAL_UART_TRANSMIT(&huart5,debugData,sizeof(*debugData),1000);
				break;
			default:
		}
	}
}
