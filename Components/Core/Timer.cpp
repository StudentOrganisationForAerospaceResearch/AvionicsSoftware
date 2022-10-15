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
 * @brief Default de-constructor makes a timer that can only be polled for state
*/

Timer::~Timer()
{
	xTimerDelete(rtTimerHandle,(DEFAULT_TIMER_COMMAND_WAIT_PERIOD*2));
	SOAR_ASSERT(rtTimerHandle, "Error Occurred, Timer could not be deleted");
}


/**
 * @brief Changes this timer object's RTOS timer period, returns true on success, returns false on failure (timer command queue full)
 */
bool Timer::ChangePeriod(const uint32_t period_ms)
{
	if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
		if (xTimerStop(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
			timerState = PAUSED;
			return true;
		}
	}
	return false;
}

bool Timer::ChangePeriodAndStart(const uint32_t period_ms)
{
	if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
		timerState = COUNTING;
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
	if (xTimerStart(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
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
	if (timerState != COUNTING) {
		return false;
	}
	// Calculates the time left on the timer before it is paused
	remainingTimeBetweenPauses = rtosTimeRemaning();
	if (xTimerStop(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
		timerState = PAUSED;
		return true;
	}
	return false;
}

/**
 * @brief Restarts timer without starting to count
*/
bool Timer::ResetTimer()
{
	if (timerState == UNINITIALIZED) {
		return false;
	}
	if (xTimerReset(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
		if (xTimerStop(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
			timerState = PAUSED;
			return true;
		}
	}
	return false;
}

/**
 * @brief Restarts Timer and starts counting
*/
bool Timer::ResetTimerAndStart()
{
	if (timerState == UNINITIALIZED) {
		return false;
	}
	if (xTimerReset(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
		timerState = COUNTING;
		return true;
	}
	return false;
}

/**
 * @brief Sets timer to auto-reload if parameter is set to true, Sets timer to one shot if parameter is set to false
*/
void Timer::SetAutoReload(bool setReloadOn)
{
	if (setReloadOn == true){
		vTimerSetReloadMode(rtTimerHandle, pdTRUE);
	}
	if (setReloadOn == false){
		vTimerSetReloadMode(rtTimerHandle, pdFALSE);
	}
}

/**
 * @brief Sets timer to auto-reload if parameter is set to true, Sets timer to one shot if parameter is set to false
*/
bool Timer::GetAutoReload()
{
	if ((uxTimerGetReloadMode(rtTimerHandle)) == pdTRUE) {
		return true;
	}
	return false;
}

TimerState Timer::GetState()
{
	if (timerState == COUNTING) {
		if (xTimerIsTimerActive(rtTimerHandle) != pdFALSE){
			timerState = COUNTING;
		}
	}
	else if ((timerState == COUNTING) && (xTimerIsTimerActive(rtTimerHandle) == pdFALSE)) {
		timerState = COMPLETE;
	}
	return timerState;
}

/**
 * @brief Returns the timers' period
*/
uint32_t Timer::GetPeriod()
{
	return (TICKS_TO_MS(xTimerGetPeriod(rtTimerHandle)));
}

/**
 * @brief Returns remaining time on timer
*/
uint32_t Timer::GetRemainingTime()
{
	if (timerState == UNINITIALIZED){
		return (GetPeriod());
		}
	else if (timerState == COUNTING){
		return (rtosTimeRemaning());
	}
	else if (timerState == PAUSED){
		return remainingTimeBetweenPauses;
	}
	else {
		return 0;
	}
}



/**
 * @brief Calculates remaining time on timer
 */
uint32_t Timer::rtosTimeRemaning()
{
	remainingTime = (TICKS_TO_MS(xTimerGetExpiryTime(rtTimerHandle) - xTaskGetTickCount()));
	return remainingTime;
}
