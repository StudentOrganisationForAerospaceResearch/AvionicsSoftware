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
//#define COMPUTER_ENVIRONMENT        // Define this if we're in Windows, Linux or Mac (not when flashing on DMB)

#ifdef COMPUTER_ENVIRONMENT
#define __CC_ARM
#endif

/* System Wide Includes ------------------------------------------------------------------*/
#include <cstdint>        // For uint32_t, etc.
#include <cstdio>        // Standard c printf, vsnprintf, etc.
#ifdef COMPUTER_ENVIRONMENT// COMPUTER -------------------------------------------------
#include <cassert>        // Standard c assert, not needed except on POSIX
#include <cstdlib>        // Standard c malloc, not needed except on POSIX
#endif

#include "cmsis_os.h"    // CMSIS RTOS definitions
#include "main_avionics.hpp"  // Main avionics definitions
#include "Utils.hpp"    // Utility functions
#include "stm32f4xx_hal.h"
#include "UARTDriver.hpp"


/* Task Definitions ------------------------------------------------------------------*/
/* - Lower priority number means lower priority task ---------------------------------*/

// FLIGHT PHASE
constexpr uint8_t FLIGHT_TASK_RTOS_PRIORITY = 3;            // Priority of the flight task
constexpr uint8_t FLIGHT_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the flight task queue
constexpr uint16_t FLIGHT_TASK_STACK_DEPTH_WORDS = 512;        // Size of the flight task stack

constexpr uint16_t FLIGHT_PHASE_DISPLAY_FREQ = 1000;    // Display frequency for flight phase information

// UART TASK
constexpr uint8_t UART_TASK_RTOS_PRIORITY = 2;            // Priority of the uart task
constexpr uint8_t UART_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the uart task queue
constexpr uint16_t UART_TASK_STACK_DEPTH_WORDS = 512;    // Size of the uart task stack

// DEBUG TASK
constexpr uint8_t TASK_DEBUG_PRIORITY = 2;            // Priority of the debug task
constexpr uint8_t TASK_DEBUG_QUEUE_DEPTH_OBJS = 10;        // Size of the debug task queue
constexpr uint16_t TASK_DEBUG_STACK_DEPTH_WORDS = 512;        // Size of the debug task stack

// BAROMETER TASK
constexpr uint8_t TASK_BAROMETER_PRIORITY = 2;            // Priority of the barometer task
constexpr uint8_t TASK_BAROMETER_QUEUE_DEPTH_OBJS = 10;        // Size of the barometer task queue
constexpr uint16_t TASK_BAROMETER_STACK_DEPTH_WORDS = 512;        // Size of the barometer task stack

// IMU TASK (ACCEL/GYRO/MAGNETO)
constexpr uint8_t TASK_IMU_PRIORITY = 2;            // Priority of the barometer task
constexpr uint8_t TASK_IMU_QUEUE_DEPTH_OBJS = 10;        // Size of the barometer task queue
constexpr uint16_t TASK_IMU_STACK_DEPTH_WORDS = 512;        // Size of the barometer task stack

// GPS TASK
constexpr uint8_t TASK_GPS_PRIORITY = 2;            // Priority of the barometer task
constexpr uint8_t TASK_GPS_QUEUE_DEPTH_OBJS = 10;        // Size of the barometer task queue
constexpr uint16_t TASK_GPS_STACK_DEPTH_WORDS = 896;        // Size of the barometer task stack

// FLASH Task
constexpr uint8_t FLASH_TASK_RTOS_PRIORITY = 2;            // Priority of the flash task
constexpr uint8_t FLASH_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the flash task queue
constexpr uint16_t FLASH_TASK_STACK_DEPTH_WORDS = 512;        // Size of the flash task stack

// WATCHDOG Task
constexpr uint8_t WATCHDOG_TASK_RTOS_PRIORITY = 3;            // Priority of the watchdog task
constexpr uint8_t WATCHDOG_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the watchdog task queue
constexpr uint16_t WATCHDOG_TASK_STACK_DEPTH_WORDS = 512;        // Size of the watchdog task stack

// HDI Task
constexpr uint8_t HDI_TASK_RTOS_PRIORITY = 2;            // Priority of the HDI task
constexpr uint8_t HDI_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the HDI task queue
constexpr uint16_t HDI_TASK_STACK_DEPTH_WORDS = 256;        // Size of the HDI task stack

// TELEMETRY Task
constexpr uint8_t TELEMETRY_TASK_RTOS_PRIORITY = 2;            // Priority of the telemetry task
constexpr uint8_t TELEMETRY_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the telemetry task queue
constexpr uint16_t TELEMETRY_TASK_STACK_DEPTH_WORDS = 512;        // Size of the telemetry task stack

// PRESSURE TRANSDUCER TASK
constexpr uint8_t TASK_PRESSURE_TRANSDUCER_PRIORITY = 2;			// Priority of the pressure transducer task
constexpr uint8_t TASK_PRESSURE_TRANSDUCER_QUEUE_DEPTH_OBJS = 10;		// Size of the pressure transducer task queue
constexpr uint16_t TASK_PRESSURE_TRANSDUCER_STACK_DEPTH_WORDS = 512;		// Size of the pressure transducer task stack

// BATTERY VOLTAGE Task
constexpr uint8_t BATTERY_TASK_RTOS_PRIORITY = 2;            // Priority of the battery voltage task
constexpr uint8_t BATTERY_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the battery voltage task queue
constexpr uint16_t BATTERY_TASK_STACK_DEPTH_WORDS = 512;        // Size of the battery voltage task stack

//nor mal 500 !!!!!!!
constexpr uint32_t TELEMETRY_DEFAULT_LOGGING_RATE_MS = 50; // Default logging delay for telemetry task
constexpr uint32_t TELEMETRY_MINIMUM_LOG_PERIOD_MS = 30; // (1000/50 = 20hz) The minimum log period / max log rate

/* Flash Addresses ------------------------------------------------------------------*/
// Start of the system storage area (spans 2 sectors)
// Holds previous Rocket State, and other low-frequency state information
constexpr uint32_t SPI_FLASH_SYSTEM_SDSS_STORAGE_START_ADDR = 0x0000;
// Start of the launch key storage area (spans 1 sector)
// Always empty until launch, then filled with the launch key which
// changes the 'backup default state' to prevent accidental venting during flight
constexpr uint32_t SPI_FLASH_SYSTEM_SSS_LAUNCH_KEY_ADDR = 0x6000;
// Start of the offsets storage area (spans 2 sectors)
// Holds the storage offsets for writing to flash, and other general medium-frequency state information
constexpr uint32_t SPI_FLASH_OFFSETS_SDSS_START_ADDR = 0x8000;
// Start of the telemetry logging storage area (spans the rest of the flash)
constexpr uint32_t SPI_FLASH_LOGGING_STORAGE_START_ADDR = 0xA000;

/* System Defines ------------------------------------------------------------------*/
/* - Each define / constexpr must have a comment explaining what it is used for     */
/* - Each define / constexpr must be all-caps. Prefer constexpr unless it's a string, or a calculation (eg. mathematical expression being more readable) */
// RTOS
constexpr uint8_t DEFAULT_QUEUE_SIZE = 10;                    // Default size of the queue
constexpr uint16_t MAX_NUMBER_OF_COMMAND_ALLOCATIONS = 100;    // Let's assume ~128B per allocation, 100 x 128B = 12800B = 12.8KB

// DEBUG
constexpr uint16_t DEBUG_TAKE_MAX_TIME_MS = 500;        // Max time in ms to take the debug semaphore
constexpr uint16_t DEBUG_SEND_MAX_TIME_MS = 500;        // Max time the assert fail is allowed to wait to send header and message to HAL
constexpr uint16_t DEBUG_PRINT_MAX_SIZE = 192;            // Max size in bytes of message print buffers

// ASSERT
constexpr uint16_t ASSERT_BUFFER_MAX_SIZE = 160;        // Max size in bytes of assert buffers (assume x2 as we have two message segments)
constexpr uint16_t ASSERT_SEND_MAX_TIME_MS = 250;        // Max time the assert fail is allowed to wait to send header and message to HAL (will take up to 2x this since it sends 2 segments)
constexpr uint16_t ASSERT_TAKE_MAX_TIME_MS = 500;        // Max time in ms to take the assert semaphore
constexpr UARTDriver* const DEFAULT_ASSERT_UART_DRIVER = UART::Debug;    // UART Handle that ASSERT messages are sent over

/* System Functions ------------------------------------------------------------------*/
//- Any system functions with an implementation here should be inline, and inline for a good reason (performance)
//- Otherwise the function may have a better place in main_avionics.cpp

// Assert macro, use this for checking all possible program errors eg. malloc success etc. supports a custom message in printf format
// This is our version of the stm32f4xx_hal_conf.h 'assert_param' macro with support for optional messages
// Example Usage: SOAR_ASSERT(ptr != 0, "Pointer on loop index %d is null!", index);
#define SOAR_ASSERT(expr, ...) ((expr) ? (void)0U : soar_assert_debug(false, (const char *)__FILE__, __LINE__, ##__VA_ARGS__)) 

// SOAR_PRINT macro, acts as an interface to the print function which sends a packet to the UART Task to print data
#define SOAR_PRINT(str, ...) (print(str, ##__VA_ARGS__))

/**
 * @brief Malloc inline function, wraps malloc for multi-platform support, asserts successful allocation
 * @param size Size of data to malloc in bytes
 * @return Returns the pointer to the allocated data
*/
inline uint8_t* soar_malloc(uint32_t size) {
#ifdef COMPUTER_ENVIRONMENT
    uint8_t* ret = (uint8_t*)malloc(size);
#else
    uint8_t* ret = (uint8_t*)pvPortMalloc(size);
#endif
    
    SOAR_ASSERT(ret, "soar_malloc failed");
    return ret;
}

/**
 * @brief Free inline function, wraps free for multi-platform support
 * @param ptr Pointer to the data to free
 */
inline void soar_free(void* ptr) {
#ifdef COMPUTER_ENVIRONMENT
    free(ptr);
#else
    vPortFree(ptr);
#endif
}

/* STM32 HAL C++ Wrappers ------------------------------------------------------------------*/


/* Other ------------------------------------------------------------------*/
// Override the new and delete operator to ensure heap4 is used for dynamic memory allocation
inline void* operator new(size_t size) { return soar_malloc(size); }
inline void operator delete(void* ptr) { soar_free(ptr); }

#endif // SOAR_MAIN_SYSTEM_DEFINES_H
