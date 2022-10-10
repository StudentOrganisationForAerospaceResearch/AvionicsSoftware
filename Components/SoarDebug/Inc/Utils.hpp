/**
 ******************************************************************************
 * File Name          : Utils.hpp
 * Description        : Utility functions
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

// Utility functions
namespace Utils
{
	uint16_t averageArray(uint16_t array[], int size);
	void writeInt32ToArray(uint8_t* array, int startIndex, int32_t value);
	void readUInt32FromUInt8Array(uint8_t* array, int startIndex, int32_t* value);
}


#endif	// AVIONICS_INCLUDE_SOAR_UTILS_HPP_
