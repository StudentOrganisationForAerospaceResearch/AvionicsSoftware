/**
 ******************************************************************************
 * File Name          : Timer.cpp
 * Description        : FreeRTOS Timer Wrapper
 ******************************************************************************
*/
#include "SystemDefines.hpp"
#include "Timer.hpp"

/**
 * @brief Empty callback function, used internally for default polling timers
*/
void empty_callback(TimerHandle_t rtTimerHandle) {};

/**
 * @brief Default constructor makes a timer that can only be polled for state
*/
Timer::Timer()
{
	// We make a timer named "Timer" with a callback function that does nothing, Autoreload false, and the default period of 1s.
	// The timer ID is specified as (void *)this to provide a unique ID for each timer object - however this is not necessary for polling timers.
	// The timer is created in the dormant state.
	rtTimerHandle = xTimerCreate("Timer", DEFAULT_TIMER_PERIOD, pdFALSE, (void *)this, empty_callback);
}

/**
 * @brief Changes this timer object's RTOS timer period, returns true on success, returns false on failure (timer command queue full)
 */
bool Timer::ChangePeriod(const uint32_t period)
{
	if (xTimerChangePeriod(rtTimerHandle, period / portTICK_RATE_MS, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE)
		return true;
	return false;
}
