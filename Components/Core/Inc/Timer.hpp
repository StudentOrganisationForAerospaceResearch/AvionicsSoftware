/**
 ******************************************************************************
 * File Name          : Timer.hpp
 * Description        : Wrapper for FreeRTOS Timers
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_TIMER_H
#define AVIONICS_INCLUDE_SOAR_CORE_TIMER_H
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Utils.hpp"

/* Macros --------------------------------------------------------------------*/
constexpr uint32_t DEFAULT_TIMER_COMMAND_WAIT_PERIOD = MS_TO_TICKS(15); // Default time to block a task if a command cannot be issued to the timer

#define DEFAULT_TIMER_PERIOD (MS_TO_TICKS(1000)) // 1s

/* Class -----------------------------------------------------------------*/

/**
 * @brief Timer class
 *
 * Wrapper for FreeRTOS Timers
*/
class Timer
{
public:
	Timer();

	bool ChangePeriod(const uint32_t period);

	// WORK-IN-PROGRESS
	// NOTES:
	// - I can think of several timer types
	// 1) Default Ctor Timer (1 second polling timer that requires polling to acquire state with no callback)
	// 2) Callback Enabled Timer (user-provided callback)



protected:
	TimerHandle_t rtTimerHandle;

};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_COMMAND_H */