/**
 ******************************************************************************
 * File Name          : Main.cpp
 * Description        : This file acts as an interface supporting CubeIDE Codegen
	while having a clean interface for development.
 ******************************************************************************
*/
#include <cstdarg>		// Support for va_start and va_end
#include <cstring>		// Support for strlen and strcpy

#include "SystemDefines.hpp"
#include "main_avionics.hpp"
#include "Mutex.hpp"

#include "FlightTask.hpp"
#include "stm32f4xx_hal_uart.h"

/* Interface Functions ------------------------------------------------------------*/

/**
 * @brief Main function interface, called inside main.cpp before os initialization takes place.
*/
void run_main() {
	// Note the errors, may need to implement newlib
	// https://stackoverflow.com/questions/19258847/stm32-c-operator-new-coide
	// Init Tasks
	FlightTask::Inst().InitTask();
	Mutex mtx;

	
	// Start the Scheduler
	// Guidelines:
	// - Be CAREFUL with race conditions after osKernelStart
	// - Recommended to not use new and delete after this point
    osKernelStart();

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

	// We have an assert fail, we pause everything else and take full control of the system
	// Be careful! If using this code in for a standard debug function ALL calls to any vsnprint functions MUST have a mutex lock/unlock
	vTaskSuspendAll();

	// This doesn't work, we don't have __sbrk inside the ARM env.
	// Look into newlib, or make our own version of __sbrk / snprintf etc. to parse the VA list
	// https://nadler.com/embedded/newlibAndFreeRTOS.html


	//// Print out the assertion message through the supported interface, we don't have a UART task running, so we directly use HAL
	//uint8_t header_buf[160] = {};
	//int16_t res = snprintf(reinterpret_cast<char*>(header_buf), 160 - 1, "-- ASSERTION FAILED --\r\nFile [%s]\r\nLine # [%d]\r\n", file, line);
	//if(res < 0) {
	//	// If we failed to generate the header, just format the line number
	//	snprintf(reinterpret_cast<char*>(header_buf), 160 - 1, "-- ASSERTION FAILED --\r\nFile [PATH_TOO_LONG]\r\nLine # [%d]\r\n", line);
	//}

	//// Output the header to the debug port
	//HAL_UART_Transmit(DEFAULT_ASSERT_UART_HANDLE, header_buf, strlen(reinterpret_cast<char*>(header_buf)), 250);

	//// Extract the string into a new buffer, and null terminate it
	//if(str != nullptr) {
	//	uint8_t str_buffer[160] = {};
	//	va_list argument_list;
	//	va_start(argument_list, str);
	//	int16_t buflen = vsnprintf(reinterpret_cast<char*>(str_buffer), sizeof(str_buffer) - 1, str, argument_list);
	//	va_end(argument_list);
	//	if(buflen > 0) {
	//		str_buffer[buflen] = '\0';
	//		HAL_UART_Transmit(DEFAULT_ASSERT_UART_HANDLE, str_buffer, buflen, 250);
	//	}
	//}

	HAL_NVIC_SystemReset();
}

