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
		xTaskCreate((TaskFunction_t)UARTTask::RunTask,
			(const char*)"UARTTask",
			(uint16_t)UART_TASK_STACK_SIZE,
			(void*)this,
			(UBaseType_t)UART_TASK_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	SOAR_ASSERT(rtValue == pdPASS, "UARTTask::InitTask() - xTaskCreate() failed");

	// Configure UART

}

void UARTTask::Run(void * pvParams)
{
	while(1) {
		Command cm;

		//Wait forever for a command
		qEvtQueue->ReceiveWait(cm);
		
		//Process the command
		HandleCommand(cm);
	}
}

void UARTTask::HandleCommand(Command& cm)
{
	switch (cm.GetCommand()) {
	case DATA_COMMAND: {
		switch (cm.GetTaskCommand()) {
		case UART_TASK_COMMAND_SEND_DEBUG:
			HAL_UART_Transmit(SystemHandles::UART_Debug, cm.GetDataPointer(), cm.GetDataSize(), DEBUG_SEND_MAX_TIME_MS);
			break;
		default:
			SOAR_PRINT("UARTTask - Received Unsupported DATA_COMMAND {%d}\r\n", cm.GetTaskCommand());
			break;
		}
	}
	case TASK_SPECIFIC_COMMAND: {
		break;
	}
	default:
		SOAR_PRINT("UARTTask - Received Unsupported Command {%d}\r\n", cm.GetCommand());
		break;
	}
}
