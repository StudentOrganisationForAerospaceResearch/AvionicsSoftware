/**
  ******************************************************************************
  * @file           : SystemDefines.hpp
  * @brief          : Macros and wrappers
  ******************************************************************************
  *
  * Contains system wide macros, defines, and wrappers
  *
  ******************************************************************************
  */
#ifndef SOAR_MAIN_SYSTEM_DEFINES_H
#define SOAR_MAIN_SYSTEM_DEFINES_H

/* Environment Defines ------------------------------------------------------------------*/
//#define POSIX_ENVIRONMENT		// Define this if we're in Windows or Linux

#ifdef POSIX_ENVIRONMENT
#define __CC_ARM
#endif

/* System Wide Includes ------------------------------------------------------------------*/
#include <cstdint>		// For uint32_t, etc.
#include <cassert>		// Standard c assert, not needed except on POSIX
#include <cstdio>		// Standard c printf, not needed except on POSIX
#include <cstdlib>		// Standard c malloc, not needed except on POSIX


#include "cmsis_os.h"	// CMSIS RTOS definitions
#include "main_avionics.hpp"  // Main avionics definitions

/* Type Definitions ------------------------------------------------------------------*/

/* System Defines ------------------------------------------------------------------*/
/* - Each define / constexpr must have a comment explaining what it is used for     */
/* - Each define / constexpr must be all-caps. Prefer constexpr unless it's a string, or a calculation (eg. mathematical expression being more readable) */

constexpr uint8_t DEFAULT_QUEUE_SIZE = 10;					// Default size of the queue
constexpr uint16_t MAX_NUMBER_OF_COMMAND_ALLOCATIONS = 100;	// Let's assume ~128B per allocation, 100 x 128B = 12800B = 12.8KB
constexpr uint8_t MAX_DEBUG_MESSAGE_LENGTH = 100;			// Max length of a debug message, not including null terminator

constexpr UART_HandleTypeDef* const DEFAULT_ASSERT_UART_HANDLE = &SystemHandles::huart5;	// Default Assert Failed UART Handle

/* System Functions ------------------------------------------------------------------*/
//- Any system functions with an implementation here should be inline, and inline for a good reason (performance)
//- Otherwise the function may have a better place in main_avionics.cpp

// Assert macro, use this for checking all possible program errors eg. malloc success etc. supports a custom message in printf format
// Example Usage: SOAR_ASSERT(ptr != 0, "Pointer on loop index %d is null!", index);
#define SOAR_ASSERT(expr, ...) ((expr) ? (void)0U : soar_assert_debug(false, (uint8_t *)__FILE__, __LINE__, ##__VA_ARGS__))



/**
 * @brief Malloc inline function, wraps malloc for multi-platform support, asserts successful allocation
 * @param size Size of data to malloc in bytes
 * @return Returns the pointer to the allocated data
*/
inline uint8_t* Malloc(uint32_t size) {
#ifdef POSIX_ENVIRONMENT
	uint8_t* ret = (uint8_t*)malloc(size);
#else
	uint8_t* ret = (uint8_t*)pvPortMalloc(size);
#endif
	
	SOAR_ASSERT(ret, "Malloc failed");
	return ret;
}

/* STM32 HAL C++ Wrappers ------------------------------------------------------------------*/


/* Other ------------------------------------------------------------------*/

#endif // SOAR_MAIN_SYSTEM_DEFINES_H
