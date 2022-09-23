/**
  ******************************************************************************
  * File Name          : Debug.c
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "DebugTask.hpp"


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void DebugTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == NULL, "Cannot initialize Debug task twice");

	// Start the task
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)DebugTask::RunTask,
			(const char*)"DebugTask",
			(uint16_t)TASK_DEBUG_STACK_SIZE,
			(void*)this,
			(UBaseType_t)TASK_DEBUG_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	//Ensure creation succeded
	SOAR_ASSERT(rtValue == pdPASS, "UARTTask::InitTask() - xTaskCreate() failed");
}

// TODO: Only run thread when appropriate GPIO pin pulled HIGH
void DebugTask::Run(void * pvParams)
{
    uint32_t prevWakeTime = osKernelSysTick();
    //uint8_t buffer = 0x00;

	while (1) {
		osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);

		//HAL_UART_Receive(&huart5, &buffer, 1, 1000); // This should be in UART Task (UART Task's job is to poll these buffers)
	}
}
