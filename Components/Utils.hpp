/**
 ******************************************************************************
 * File Name          : Utils.hpp
 * Description        : Utility functions, accessible system wide.
 *				Includes functions for converting between data types
 *				Use specific naming, or namespace to avoid conflicts
 *				Keep namespace Utils to avoid conflicts with other libraries
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_UTILS_HPP_
#define AVIONICS_INCLUDE_SOAR_UTILS_HPP_
#include "cmsis_os.h"	// CMSIS RTOS definitions

// Math macros and conversions
constexpr double MATH_PI = 3.14159265358979323846;
#define DEG_TO_RAD(degrees) ((degrees) * 0.01745329251994329576923690768489f)	// Degrees to radians (PI/180)
#define RAD_TO_DEG(radians) ((radians) * 57.295779513082320876798154814105f)	// Radians to degrees (180/PI)

// Conversion macros (SYSTEM)
#define TICKS_TO_MS(time_ticks) ((time_ticks) * 1000 / osKernelSysTickFrequency) // System ticks to milliseconds
#define MS_TO_TICKS(time_ms) ((time_ms) * osKernelSysTickFrequency / 1000) // Milliseconds to system ticks

// System Time Macros
constexpr uint32_t MAX_DELAY_MS = TICKS_TO_MS(portMAX_DELAY);
constexpr uint32_t MAX_DELAY_TICKS = portMAX_DELAY;

// Utility functions
namespace Utils
{
	// Arrays
	uint16_t averageArray(uint16_t array[], int size);
	void writeInt32ToArray(uint8_t* array, int startIndex, int32_t value);
	void readUInt32FromUInt8Array(uint8_t* array, int startIndex, int32_t* value);

	// CRC
	uint32_t getCRC32(uint8_t* data, uint32_t size);

	// String Manipulation
	inline bool IsAsciiNum(uint8_t c) { return (c >= '0' && c <= '9'); }
	inline bool IsAsciiChar(uint8_t c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }
	inline bool IsAsciiLowercase(uint8_t c) { return (c >= 'a' && c <= 'z'); }

}


#endif	// AVIONICS_INCLUDE_SOAR_UTILS_HPP_
