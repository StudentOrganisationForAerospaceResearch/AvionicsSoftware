#include "SystemDefines.hpp"
#include "Timer.hpp"
/**
 ******************************************************************************
 * File Name          : Timer.cpp
 * Description        : FreeRTOS Timer Wrapper
 ******************************************************************************
*/

/**
 * @brief Empty callback function, used internally for default polling timers
*/
void empty_callback(TimerHandle_t rtTimerHandle) {};

//CurrentState Timer::completed()
//{
//	timerState = COMPLETE;
//}

/**
 * @brief Default constructor makes a timer that can only be polled for state
*/
Timer::Timer()
{
	// We make a timer named "Timer" with a callback function that does nothing, Autoreload false, and the default period of 1s.
	// The timer ID is specified as (void *)this to provide a unique ID for each timer object - however this is not necessary for polling timers.
	// The timer is created in the dormant state.
	assert (rtTimerHandle = xTimerCreate("Timer", DEFAULT_TIMER_PERIOD, pdFALSE, (void *)this, empty_callback));
	timerState = UNINITIALIZED;
}

/**
 * @brief Changes this timer object's RTOS timer period, returns true on success, returns false on failure (timer command queue full)
 */
bool Timer::ChangePeriod(const uint32_t period)
{
	if (xTimerChangePeriod(rtTimerHandle, period / portTICK_RATE_MS, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
		timerState = COUNTING;
		return true;
	}
	return false;
}


bool Timer::StartTimer()
{
	if (xTimerStart(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
		timerState = COUNTING;
		return true;
	}
	return false;
}

bool Timer::StopTimer()
{
	if (xTimerStart(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
		timerState = PAUSED;
		return true;
	}
	return false;
}

CurrentState Timer::completed(TimerHandle_t xTimer)
{
	if ((remainingTime = TICKS_TO_MS(xTimerGetExpiryTime(rtTimerHandle) - xTaskGetTickCount()) == 0)) {
		return COMPLETE;
	}
}

CurrentState Timer::GetState(TimerHandle_t xTimer)
{
	if (xTimerIsTimerActive(rtTimerHandle) != pdFALSE)
		return COUNTING;
	return (completed(rtTimerHandle));
	return timerState;
}



