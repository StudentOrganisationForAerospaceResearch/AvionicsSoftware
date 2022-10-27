/**
  ******************************************************************************
  * File Name          : BarometerTask.cpp
  *
  *	Source Info		   : Based on Andromeda V3.31 Legacy Implementation
  *						 Andromeda_V3.31_Legacy/Core/Src/ReadBarometer.c
  *
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the MS5607-02BA03 barometer on the flight board. A
  *                      thread task is included that will constantly loop,
  *                      reading and updating the passed BarometerData struct.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "BarometerTask.hpp"


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void BarometerTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Debug task twice");

	// Start the task
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)BarometerTask::RunTask,
			(const char*)"BaroTask",
			(uint16_t)TASK_BAROMETER_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)TASK_BAROMETER_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	//Ensure creation succeded
	SOAR_ASSERT(rtValue == pdPASS, "BarometerTask::InitTask() - xTaskCreate() failed");
}

void BarometerTask::Run(void * pvParams)
{
	while (1) {

		osDelay(100);
		
	}
}
