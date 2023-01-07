/**
 ******************************************************************************
 * File Name          : UARTTask.cpp
 * Description        : UART
 ******************************************************************************
*/

#include "UARTTask.hpp"

/**
 * TODO: Currently not used, would be used for DMA buffer configuration or interrupt setup
 * @brief Configures UART DMA buffers and interrupts
 * 
*/
void UARTTask::ConfigureUART()
{
	// UART 5 - Uses polling for now (switch to DMA or interrupts once SOAR-Protocol is defined)
}

/**
 * @brief Initializes UART task with the RTOS scheduler
*/
void UARTTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize UART task twice");
	
	// Start the task
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)UARTTask::RunTask,
			(const char*)"UARTTask",
			(uint16_t)UART_TASK_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)UART_TASK_RTOS_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	//Ensure creation succeded
	SOAR_ASSERT(rtValue == pdPASS, "UARTTask::InitTask() - xTaskCreate() failed");

	// Configure DMA
	 
}

/**
 * @brief Instance Run loop for the UART Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
*/
void UARTTask::Run(void * pvParams)
{
	//UART Task loop
	while(1) {
		Command cm;

		//Wait forever for a command
		qEvtQueue->ReceiveWait(cm);
		
		//Process the command
		HandleCommand(cm);
	}
}

/**
 * @brief HandleCommand handles any command passed to the UART task primary event queue. Responsible for
 * 		  handling all commands, even if unsupported. (Unexpected commands must still be reset) 
 * @param cm Reference to the command object to handle
*/
void UARTTask::HandleCommand(Command& cm)
{
	//Switch for the GLOBAL_COMMAND
	switch (cm.GetCommand()) {
	case DATA_COMMAND: {
		//Switch for task specific command within DATA_COMMAND
		switch (cm.GetTaskCommand()) {
		case UART_TASK_COMMAND_SEND_DEBUG:
			HAL_UART_Transmit(SystemHandles::UART_Debug, cm.GetDataPointer(), cm.GetDataSize(), DEBUG_SEND_MAX_TIME_MS);
			break;
		default:
			SOAR_PRINT("UARTTask - Received Unsupported DATA_COMMAND {%d}\n", cm.GetTaskCommand());
			break;
		}
	}
	case TASK_SPECIFIC_COMMAND: {
		break;
	}
	default:
		SOAR_PRINT("UARTTask - Received Unsupported Command {%d}\n", cm.GetCommand());
		break;
	}

	//No matter what we happens, we must reset allocated data
	cm.Reset();
}
