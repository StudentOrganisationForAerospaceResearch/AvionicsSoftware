/**
 ******************************************************************************
 * File Name          : main_avionics.cpp
 * Description        : This file acts as an interface supporting CubeIDE Codegen
	while having a clean interface for development.
 ******************************************************************************
*/
#include <cstdarg>		// Support for va_start and va_end
#include <cstring>		// Support for strlen and strcpy

#include "SystemDefines.hpp"
#include "main_avionics.hpp"
#include "stm32f4xx_hal_uart.h"
#include "Mutex.hpp"
#include "Command.hpp"

// Tasks
#include "UARTTask.hpp"
#include "FlightTask.hpp"
#include "DebugTask.hpp"
#include "Timer.hpp"


/* Global Variables ------------------------------------------------------------------*/
Mutex Global::vaListMutex;

void test_timer_state () {
    Timer testTimer1;
    SOAR_PRINT("Expected Output: 0 (UNINITIALIZED)\n");
    SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

    SOAR_PRINT("Starting Timer...\n");
    testTimer1.StartTimer();
    SOAR_PRINT("Expected Output: 1 (COUNTING)\n");
	SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

	SOAR_PRINT("Stopping Timer...\n");
	testTimer1.StopTimer();
	SOAR_PRINT("Expected Output: 2 (PAUSED)\n");
	SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

	osDelay(1100);
	SOAR_PRINT("Expected Output: 3 (COMPLETE)\n");
	SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_destructor() {
	SOAR_PRINT("\n Testing destructor...\n");
	SOAR_PRINT("Expected Output: 'TIMER HAS BEEN DELETED'\n");
	{
		Timer testTimer2;
	}
}

void test_period () {
	Timer testTimer3;

	testTimer3.StartTimer();
	SOAR_PRINT("\n Testing GetPeriod function...\n");
	SOAR_PRINT("Expected Output: 1000 ms\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetPeriod());

	SOAR_PRINT("Changing Period to 5000ms\n");
	SOAR_PRINT("Expected Result: 5000ms\n");
	SOAR_PRINT("Actual Result: %d\n\n", testTimer3.GetPeriod());

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 2 (PAUSED)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetState());

	SOAR_PRINT("Testing to see if the timer restarts if timer period is changed\n");
	SOAR_PRINT("Using GetPeriod function ...\n");
	SOAR_PRINT("Expected Output: 5000 ms\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetRemainingTime());

	SOAR_PRINT("Testing ChangePeriodAndStart function...\n");
	testTimer3.ChangePeriodAndStart(7000);
	SOAR_PRINT("Expected Output: 7000 ms\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetPeriod());

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 1 (COUNTING)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetState());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_autoreload () {
	Timer testTimer4;

	SOAR_PRINT("\n Testing GetAutoReload function...\n");
	SOAR_PRINT("Expected Output: 'Timer is set to autoreload'\n");
	if (testTimer4.GetAutoReload() == true) {
		SOAR_PRINT("Actual Output : 'Timer is set to autoreload'\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("Changing timer to one-shot...\n");
	testTimer4.SetAutoReload(false);
	SOAR_PRINT("Expected Output: 'Timer is set to one-shot'\n");
	if (testTimer4.GetAutoReload() == false) {
		SOAR_PRINT("Actual Output : 'Timer is set to one-shot'\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_reset(){
	Timer testTimer5;
	SOAR_PRINT("\n Trying to reset timer in UNINITIALIZED state\n");
	SOAR_PRINT("Expected Output: Timer did not reset\n");
	if (testTimer5.ResetTimer() == false) {
		SOAR_PRINT("Actual Output: Timer did not reset\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	testTimer5.StartTimer();
	osDelay(300);
	SOAR_PRINT("Testing reset function...\n");
	testTimer5.ResetTimer();
	SOAR_PRINT("Expected Result: Timer Reset Successfully\n");
	if (testTimer5.ResetTimer() == true) {
		SOAR_PRINT("Actual Output: Timer Reset Successfully\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 2 (PAUSED)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer5.GetState());

	SOAR_PRINT("Testing ResetTimerAndStart function..\n");
	if (testTimer5.ResetTimerAndStart() == true) {
		SOAR_PRINT("Actual Output: Timer Reset Successfully\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 1 (COUNTING)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer5.GetState());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_get_time () {
	Timer testTimer6;

	SOAR_PRINT("Testing GetRemainingTime function ...\n");
	SOAR_PRINT("Expected Output: 1000 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	osDelay(300);
	SOAR_PRINT("Expected Output: 700 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	osDelay(300);
	testTimer6.StopTimer();
	SOAR_PRINT("Expected Output: 400 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	osDelay(500);
	SOAR_PRINT("Expected Output: 0 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

/* Interface Functions ------------------------------------------------------------*/
/**
 * @brief Main function interface, called inside main.cpp before os initialization takes place.
*/
void run_main() {
	// Init Tasks
	FlightTask::Inst().InitTask();
	UARTTask::Inst().InitTask();
	DebugTask::Inst().InitTask();

	// Print System Boot Info : Warning, don't queue more than 10 prints before scheduler starts
	SOAR_PRINT("\n-- SOAR AVIONICS --\n");
	SOAR_PRINT("System Reset Reason: [TODO]\n"); //TODO: If we want a system reset reason we need to save it on flash
	SOAR_PRINT("Current System Heap Use: %d Bytes\n", xPortGetFreeHeapSize());
	SOAR_PRINT("Lowest Ever Heap Size: %d Bytes\n\n", xPortGetMinimumEverFreeHeapSize());
	
	// Start the Scheduler
	// Guidelines:
	// - Be CAREFUL with race conditions after osKernelStart
	// - Recommended to not use new and delete after this point
    osKernelStart();

    test_timer_state();
    test_destructor();
    test_period ();
    test_autoreload ();
    test_reset();
    test_get_time ();

	// Should never reach here
	SOAR_ASSERT(false, "osKernelStart() failed");

	while (1)
	{
		osDelay(100);
	}
}

/**
 * @brief Called by RunDefaultTask inside main.cpp, if using task implement here and change the location for user code in main.cpp.
*/
void run_StartDefaultTask()
{
    SOAR_ASSERT(false, "Default task is not used");
}


/* System Functions ------------------------------------------------------------*/

/**
* @brief Variadic print function, sends a command packet to the queue
* @param str String to print with printf style formatting
* @param ... Additional arguments to print if assertion fails, in same format as printf
*/
void print(const char* str, ...)
{
	//Try to take the VA list mutex
	if (Global::vaListMutex.Lock(DEBUG_TAKE_MAX_TIME_MS)) {
		// If we have a message, and can use VA list, extract the string into a new buffer, and null terminate it
		uint8_t str_buffer[DEBUG_PRINT_MAX_SIZE] = {};
		va_list argument_list;
		va_start(argument_list, str);
		int16_t buflen = vsnprintf(reinterpret_cast<char*>(str_buffer), sizeof(str_buffer) - 1, str, argument_list);
		va_end(argument_list);
		if (buflen > 0) {
			str_buffer[buflen] = '\0';
		}

		// Release the VA List Mutex
		Global::vaListMutex.Unlock();

		//Generate a command
		Command cmd(DATA_COMMAND, (uint16_t)UART_TASK_COMMAND_SEND_DEBUG); // Set the UART channel to send data on
		
		//Copy data into the command
		cmd.CopyDataToCommand(str_buffer, buflen);
			
		//Send this packet off to the UART Task
		UARTTask::Inst().GetEventQueue()->Send(cmd);
	}
	else
	{
		//TODO: Print out that we could not acquire the VA list mutex
		SOAR_ASSERT(false, "Could not acquire VA_LIST mutex");
	}
}

/**
 * @brief Variadic assertion function, wraps assert for multi-platform support and debug builds
 * @param condition Assertion that this condition is true (!0)
 * @param file File that the assertion is in (__FILE__)
 * @param line Line number that the assertion is on (__LINE__)
 * @param str Optional message to print if assertion fails. Must be less than 192 characters AFTER formatting
 * @param ... Additional arguments to print if assertion fails, in same format as printf
 */
void soar_assert_debug(bool condition, const char* file, const uint16_t line, const char* str, ...) {
	// If assertion succeeds, do nothing
	if (condition) {
		return;
	}

	bool printMessage = false;

	// NOTE: Be careful! If va_list funcs while RTOS is active ALL calls to any vsnprint functions MUST have a mutex lock/unlock
	// NOTE: https://nadler.com/embedded/newlibAndFreeRTOS.html

	// We have an assert fail, we try to take control of the Debug semaphore, and then suspend all other parts of the system
	if (Global::vaListMutex.Lock(ASSERT_TAKE_MAX_TIME_MS)) {
		// We have the mutex, we can now safely print the message
		printMessage = true;
	}
	
	vTaskSuspendAll();

	//If we have the vaListMutex, we can safely use vsnprintf
	if (printMessage) {
		// Print out the assertion header through the supported interface, we don't have a UART task running, so we directly use HAL
		uint8_t header_buf[ASSERT_BUFFER_MAX_SIZE] = {};
		int16_t res = snprintf(reinterpret_cast<char*>(header_buf), ASSERT_BUFFER_MAX_SIZE - 1, "\r\n\n-- ASSERTION FAILED --\r\nFile [%s] @ Line # [%d]\r\n", file, line);
		if (res < 0) {
			// If we failed to generate the header, just format the line number
			snprintf(reinterpret_cast<char*>(header_buf), ASSERT_BUFFER_MAX_SIZE - 1, "\r\n\n-- ASSERTION FAILED --\r\nFile [PATH_TOO_LONG] @ Line # [%d]\r\n", line);
		}

		// Output the header to the debug port
		HAL_UART_Transmit(DEFAULT_ASSERT_UART_HANDLE, header_buf, strlen(reinterpret_cast<char*>(header_buf)), ASSERT_SEND_MAX_TIME_MS);

		// If we have a message, and can use VA list, extract the string into a new buffer, and null terminate it
		if (printMessage && str != nullptr) {
			uint8_t str_buffer[ASSERT_BUFFER_MAX_SIZE] = {};
			va_list argument_list;
			va_start(argument_list, str);
			int16_t buflen = vsnprintf(reinterpret_cast<char*>(str_buffer), sizeof(str_buffer) - 1, str, argument_list);
			va_end(argument_list);
			if (buflen > 0) {
				str_buffer[buflen] = '\0';
				HAL_UART_Transmit(DEFAULT_ASSERT_UART_HANDLE, str_buffer, buflen, ASSERT_SEND_MAX_TIME_MS);
			}
		}
	}
	else {
		//TODO: Should manually print out the assertion header
		HAL_UART_Transmit(DEFAULT_ASSERT_UART_HANDLE, (uint8_t*)"-- ASSERTION FAILED --\r\nCould not acquire vaListMutex\r\n", 55, ASSERT_SEND_MAX_TIME_MS);
	}

	HAL_NVIC_SystemReset();

	// We should not reach this code, but if we do, we should resume the scheduler
	xTaskResumeAll();
}

