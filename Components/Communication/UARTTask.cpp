#include "UARTTask.hpp"
#include "Inc/UARTTask.hpp"
/**
 ******************************************************************************
 * File Name          : UARTTask.cpp
 * Description        : UART
 ******************************************************************************
*/

void UARTTask::ConfigureUART()
{
	// UART 5 - Uses Interrupts for now (switch to DMA once SOAR-Protocol is defined)

	
}

void UARTTask::InitTask()
{
	// Start the task
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)UARTTask::Run,
			(const char*)"UARTTask",
			(uint16_t)UART_TASK_STACK_SIZE,
			(void*)this,
			(UBaseType_t)UART_TASK_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	SOAR_ASSERT(rtValue == pdPASS, "UARTTask::InitTask() - xTaskCreate() failed");

	// Configure UART

}

void UARTTask::Run(void* pvParams)
{
	while(1) {

		// Replace this with HAL_UART_Receive or something to check DMA buffers based on interrupt flags etc.
		osDelay(100);

		// 
	}
}


