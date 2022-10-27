/**
  ******************************************************************************
  * File Name          : Debug.cpp
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "DebugTask.hpp"
#include "Command.hpp"
#include "Utils.hpp"
#include <cstring>

#include "GPIO.hpp"
#include "stm32f4xx_hal.h"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* HAL Callbacks ----------------------------------------------------------------*/
/**
 * @brief HAL Callback for DMA/Interrupt Complete
 *
 * TODO: This should eventually be in DMAController/main_avionics/UARTTask depending on how many tasks use DMA vs Interrupt vs Polling
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart->Instance == SystemHandles::UART_Debug->Instance)
		DebugTask::Inst().InterruptRxData();
}

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Constructor, sets all member variables
 */
DebugTask::DebugTask() : Task(TASK_DEBUG_STACK_DEPTH_WORDS)
{
	memset(debugBuffer, 0, sizeof(debugBuffer));
	debugMsgIdx = 0;
	isDebugMsgReady = false;
}

/**
 * @brief Init task for RTOS
 */
void DebugTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Debug task twice");

	// Start the task
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)DebugTask::RunTask,
			(const char*)"DebugTask",
			(uint16_t)TASK_DEBUG_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)TASK_DEBUG_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	//Ensure creation succeded
	SOAR_ASSERT(rtValue == pdPASS, "DebugTask::InitTask - xTaskCreate() failed");
}

// TODO: Only run thread when appropriate GPIO pin pulled HIGH (or by define)
/**
 *	@brief Runcode for the DebugTask
 */
void DebugTask::Run(void * pvParams)
{
	// Arm the interrupt
	ReceiveData();

	while (1) {
		Command cm;

		//Wait forever for a command
		qEvtQueue->ReceiveWait(cm);

		//Process the command
		if(cm.GetCommand() == DATA_COMMAND && cm.GetTaskCommand() == EVENT_DEBUG_RX_COMPLETE) {
			HandleDebugMessage((const char*)debugBuffer);
		}

		cm.Reset();
	}
}

/**
 * @brief Handles debug messages, assumes msg is null terminated
 * @param msg Message to read, must be null termianted
 */
void DebugTask::HandleDebugMessage(const char* msg)
{
	if (strcmp(msg, "sysreset") == 0) {
		// Reset the system
		SOAR_ASSERT(false, "System reset requested");
	}
	else if (strcmp(msg, "sysinfo") == 0) {
		// Print message
		SOAR_PRINT("\n\t-- SOAR System Info --\n");
		SOAR_PRINT("Current System Heap Use: %d Bytes\n", xPortGetFreeHeapSize());
		SOAR_PRINT("Lowest Ever Heap Size\t: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
		SOAR_PRINT("Debug Task Runtime  \t: %d ms\n\n", TICKS_TO_MS(xTaskGetTickCount()));
	}
	else if (strcmp(msg, "blinkled") == 0) {
		// Print message
		SOAR_PRINT("Debug 'LED blink' command requested\n");
		GPIO::LED1::On();
		// TODO: Send to HID task to blink LED, this shouldn't delay
	}
	else {
		// Single character command, or unknown command
		switch (msg[0]) {
		default:
			SOAR_PRINT("Debug, unknown command: %s\n", msg);
			break;
		}
	}

	//We've read the data, clear the buffer
	debugMsgIdx = 0;
	isDebugMsgReady = false;
}

/**
 * @brief Receive data, currently receives by arming interrupt
 */
bool DebugTask::ReceiveData()
{
	HAL_UART_Receive_IT(SystemHandles::UART_Debug, &debugRxChar, 1);
	return true;
}

/**
 * @brief Receive data to the buffer
 * @return Whether the debugBuffer is ready or not
 */
void DebugTask::InterruptRxData()
{
	// If we already have an unprocessed debug message, ignore this byte
	if (!isDebugMsgReady) {
		// Check byte for end of message - note if using termite you must turn on append CR
		if (debugRxChar == '\r' || debugMsgIdx == DEBUG_RX_BUFFER_SZ_BYTES) {
			// Null terminate and process
			debugBuffer[debugMsgIdx++] = '\0';
			isDebugMsgReady = true;

			// Notify the debug task
			Command cm(DATA_COMMAND, EVENT_DEBUG_RX_COMPLETE);
			bool res = qEvtQueue->SendFromISR(cm);

			// If we failed to send the event, we should reset the buffer, that way DebugTask doesn't stall
			if (res == false) {
				debugMsgIdx = 0;
				isDebugMsgReady = false;
			}
		}
		else {
			debugBuffer[debugMsgIdx++] = debugRxChar;
		}
	}

	//Re-arm the interrupt
	ReceiveData();
}
