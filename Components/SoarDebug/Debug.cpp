/**
  ******************************************************************************
  * File Name          : Debug.c
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Debug.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

// TODO: Only run thread when appropriate GPIO pin pulled HIGH
void debugTask(void const* arg) {
    uint32_t prevWakeTime = osKernelSysTick();
    //uint8_t buffer = 0x00;

	while (1) {
		osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);

		//HAL_UART_Receive(&huart5, &buffer, 1, 1000); // This should be in UART Task (UART Task's job is to poll these buffers)
	}
}
