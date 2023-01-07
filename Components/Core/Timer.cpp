#include "SystemDefines.hpp"
#include "Timer.hpp"

/**
 ******************************************************************************
 * File Name          : Timer.cpp
 * Description        : FreeRTOS Timer Wrapper
 ******************************************************************************
*/

/**
 * Default constructor makes a timer that can only be polled for state
 * Default behaviour : ->Autoreload is set to false (One shot Timer)
 * 					   ->Timer Period is 1000ms
 * 					   ->The Callback function simply changes state to COMPLETE and has no other functionality
*/
Timer::Timer()
{
	// We make a timer named "Timer" with a callback function that does nothing, Autoreload false, and the default period of 1s.
	// The timer ID is specified as (void *)this to provide a unique ID for each timer object - however this is not necessary for polling timers.
	// The timer is created in the dormant state.
	rtTimerHandle = xTimerCreate("Timer", DEFAULT_TIMER_PERIOD, pdFALSE, (void *)this, CallbackFunction);
	SOAR_ASSERT(rtTimerHandle, "Error Occurred, Timer not created");
	timerState = UNINITIALIZED;
}

/**
 * Constructor for callback enabled timer
 * ! User has to add "Timer::CallbackFunction(rtTimerHandle);" in the callback function for accurate functioning of Timer States
 * Default behaviour : ->Autoreload is set to false (One shot Timer)
 * 					   ->Timer Period is 1000ms
 * 					   ->The Callback function will be provided by the user while making sure to follow instruction above
*/
Timer::Timer(void (*TimerCallbackFunction_t)( TimerHandle_t xTimer ))
{
	rtTimerHandle = xTimerCreate("Timer", DEFAULT_TIMER_PERIOD, pdFALSE, (void *)this, TimerCallbackFunction_t);
	SOAR_ASSERT(rtTimerHandle, "Error Occurred, Timer not created");
	timerState = UNINITIALIZED;
}

/**
 * @brief Default de-constructor makes a timer that can only be polled for state
*/
Timer::~Timer()
{
	if (xTimerDelete(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD*2) == pdPASS) {
		SOAR_PRINT("Timer has been deleted \n\n");
	}
}

/**
 * @brief Callback function for polling timers
*/
void Timer::CallbackFunction(TimerHandle_t xTimer){
	Timer* ptrTimer = (Timer*)pvTimerGetTimerID(xTimer);
	ptrTimer->timerState = COMPLETE;
}

/**
 * @brief Changes timer period, Sets timer state back to uninitialized and stops timer
 */
bool Timer::ChangePeriod(const uint32_t period_ms)
{
	if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
		if (xTimerStop(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
			timerState = UNINITIALIZED;
			return true;
		}
	}
	return false;
}

/**
 * @brief Changes timer period, Sets timer state back to counting and starts timer
 */
bool Timer::ChangePeriodAndStart(const uint32_t period_ms)
{
	if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
		timerState = COUNTING;
		return true;
	}
	return false;
}

/**
 * @brief Starts the timer
*/
bool Timer::Start()
{
	if ((timerState == COMPLETE) || (timerState == COUNTING)) {
		return false;
	}
	else if (timerState == PAUSED) {
		ChangePeriod(remainingTimeBetweenPauses);
	}
	if (xTimerStart(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
		timerState = COUNTING;
		return true;
	}
	return false;
}

/**
 * @brief Stops the timer
*/
bool Timer::Stop()
{
	// Checks if timer is in counting state because it cannot be stopped in any other state
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
			timerState = UNINITIALIZED;
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
		//Testing purposes
		SOAR_PRINT("Set to Auto Reload\n\n");
	}
	if (setReloadOn == false){
		vTimerSetReloadMode(rtTimerHandle, pdFALSE);
		//Testing purposes
		SOAR_PRINT("Set to One Shot\n\n");
	}
}

/**
 * @brief Returns true if the timer is set to autoreload and false if it is set to one-shot
*/
bool Timer::GetAutoReload()
{
	if ((uxTimerGetReloadMode(rtTimerHandle)) == (( UBaseType_t ) pdTRUE)) {
		return true;
	}
	if ((uxTimerGetReloadMode(rtTimerHandle)) == (( UBaseType_t ) pdFALSE)) {
		return false;
	}
}

/**
 * @brief Returns timer state enum
*/
TimerState Timer::GetState()
{
	return timerState;
}

/**
 * @brief Returns the timers' period
*/
const uint32_t Timer::GetPeriod()
{
	return (TICKS_TO_MS(xTimerGetPeriod(rtTimerHandle)));
}

/**
 * @brief Returns remaining time on timer based on current state
*/
const uint32_t Timer::GetRemainingTime()
{
	if (timerState == UNINITIALIZED){
		return (GetPeriod());
		}
	else if (timerState == COUNTING){
		return rtosTimeRemaning();
	}
	else if (timerState == PAUSED){
		return remainingTimeBetweenPauses;
	}
	else {
		return 0;
	}
}


/**
 * @brief Calculates remaining time on timer in counting state
 */
uint32_t Timer::rtosTimeRemaning()
{
	remainingTime = (TICKS_TO_MS(xTimerGetExpiryTime(rtTimerHandle) - xTaskGetTickCount()));
	return remainingTime;
}
