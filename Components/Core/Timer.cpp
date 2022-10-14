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
	SOAR_ASSERT(rtTimerHandle, "Error Occurred, Timer not created");
	timerState = UNINITIALIZED;
}



/**
 * @brief Changes this timer object's RTOS timer period, returns true on success, returns false on failure (timer command queue full)
 */
bool Timer::ChangePeriod(const uint32_t period_ms)
{
	if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
		StopTimer();
		timerState = PAUSED;
		return true;
	}
	return false;
}

/**
 * @brief Starts the timer, returns true on success, returns false on failure
*/
bool Timer::StartTimer()
{
	if ((timerState == COMPLETE) || (timerState == COUNTING)) {
		return false;
	}
	else if (xTimerStart(rtTimerHandle, 0) == pdPASS) {
		timerState = COUNTING;
		return true;
	}
	return false;
}

/**
 * @brief Stops the timer, returns true on success, returns false on failure
*/
bool Timer::StopTimer()
{
	if ((timerState == COMPLETE) || (timerState == UNINITIALIZED) || (timerState == PAUSED)) {
		return false;
	}
	else if (xTimerStop(rtTimerHandle, 0) == pdPASS) {
		timerState = PAUSED;
		return true;
	}
	return false;
}


CurrentState Timer::GetState()
{
	if (xTimerIsTimerActive(rtTimerHandle) != pdFALSE){
		timerState = COUNTING;
	}
	else if ((xTimerIsTimerActive(rtTimerHandle) == pdFALSE) && (timerState == COUNTING)) {
		timerState = COMPLETE;
	}
	return timerState;
}
