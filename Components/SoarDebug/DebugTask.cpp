/**
  ******************************************************************************
  * File Name          : Debug.c
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "DebugTask.hpp"
#include <cstring>

#include "GPIO.hpp"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Constructor, sets all member variables
 */
DebugTask::DebugTask() : Task(TASK_DEBUG_STACK_DEPTH_WORDS)
{
	memset(debugBuffer, 0, sizeof(debugBuffer));
	debugMsgIdx = 0;
	isDebugMsgReady = false;
	dmaController = new DMAController(SystemHandles::UART_Debug, DEBUG_RX_BUFFER_SZ_BYTES);
}

/**
 * @brief Inits task for RTOS
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
	SOAR_ASSERT(rtValue == pdPASS, "UARTTask::InitTask() - xTaskCreate() failed");
}

// TODO: Only run thread when appropriate GPIO pin pulled HIGH
/**
 *	@brief Runcode for the DebugTask
 */
void DebugTask::Run(void * pvParams)
{
    uint32_t prevWakeTime = osKernelSysTick();
    //uint8_t buffer = 0x00;

	while (1) {
		osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);

		//Poll for Rx data, if we have a ready message, process it
		if(ReceiveData()) {
			HandleDebugMessage(reinterpret_cast<const char*>(debugBuffer));
		}
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
	else if (strcmp(msg, "sysprint") == 0) {
		// Print message
		SOAR_PRINT("Debug, 'sysprint' command requested\n");
	}
	else if (strcmp(msg, "blinkled") == 0) {
		// Print message
		SOAR_PRINT("Debug 'LED blink' command requested\n");
		GPIO::LED1::On();
		// TODO: Send to HID task to blink LED, this shouldn't delay
	}
	else {
		// Single character command
		switch (msg[0]) {
		default:
			SOAR_PRINT("Debug, unknown command: %s\n", msg);
			break;
		}
	}

	// We 'ate' this buffer
	isDebugMsgReady = false;
}

/**
 * @brief Receive data to the buffer
 * @return Whether the debugBuffer is ready or not
 */
bool DebugTask::ReceiveData()
{
	// Receive using the DMAController
	uint8_t* bufPtr = nullptr;
	uint16_t bytesRead = dmaController->BlockUntilDataRead(bufPtr);

	// If we have data, copy to our buffer
	if(bytesRead && bufPtr) {
		memcpy(debugBuffer, bufPtr, bytesRead);
		isDebugMsgReady = true;
	}
	dmaController->Release(bufPtr);
	return isDebugMsgReady;

	// Receive data using polling -- not used, won't work at high baud, may not be reliable based on system load event at low baud
	//uint8_t debugRxChar = 0;
	//HAL_UART_Receive(SystemHandles::UART_Debug, &debugRxChar, 1, MAX_DELAY_MS);

	//// Check if the debug message is ready
	//if (!isDebugMsgReady) {
	//	HAL_UART_Transmit(SystemHandles::UART_Debug, &debugRxChar, 1, 100);
	//	if (!Utils::IsAsciiChar(debugRxChar)) { // If not an ASCII number character...
	//		debugRxChar |= 0x20; // Set bit 5, so capital ASCII letters are now lowercase
	//		if (!Utils::IsAsciiLowercase(debugRxChar)) { // If not an ASCII lowercase letter...
	//			debugBuffer[debugMsgIdx] = 0; // Terminate the string
	//			debugMsgIdx = 0;
	//			isDebugMsgReady = true;
	//			return true;
	//		}
	//		debugBuffer[debugMsgIdx] = debugRxChar;
	//	}

	//	//If we have too many bytes for the buffer, parse it anyway
	//	if (debugMsgIdx++ == DEBUG_RX_BUFFER_SZ_BYTES) {
	//		isDebugMsgReady = true;
	//	}
	//}

	//return isDebugMsgReady;
}
