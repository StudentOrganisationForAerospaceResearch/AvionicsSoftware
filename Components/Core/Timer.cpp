/**
 ******************************************************************************
 * File Name          : Timer.cpp
 * Description        : FreeRTOS Timer Wrapper
 ******************************************************************************
*/
#include "SystemDefines.hpp"
#include "Timer.hpp"

/**
 * @brief Default constructor makes a timer that can only be polled for state
 * Default behaviour : ->Autoreload is set to false (One shot Timer)
 *                        ->Timer Period is 1000ms
 *                        ->The Callback function simply changes state to COMPLETE and has no other functionality
*/
Timer::Timer()
{
    // We make a timer named "Timer" with a callback function that does nothing, Autoreload false, and the default period of 1s.
    // The timer ID is specified as (void *)this to provide a unique ID for each timer object - however this is not necessary for polling timers.
    // The timer is created in the dormant state.
    rtTimerHandle = xTimerCreate("Timer", timerPeriod, pdFALSE, (void *)this, DefaultCallback);
    SOAR_ASSERT(rtTimerHandle, "Error Occurred, Timer not created");
    timerState = UNINITIALIZED;
}

/**
 * Constructor for callback enabled timer
 * ! User has to add "Timer::DefaultCallback(rtTimerHandle);" in the callback function for accurate functioning of Timer States
 * Default behaviour : ->Autoreload is set to false (One shot Timer)
 *                        ->Timer Period is 1000ms
 *                        ->The Callback function will be provided by the user while making sure to follow instruction above
*/
Timer::Timer(void (*TimerDefaultCallback_t)( TimerHandle_t xTimer ))
{
    rtTimerHandle = xTimerCreate("Timer", timerPeriod, pdFALSE, (void *)this, TimerDefaultCallback_t);
    SOAR_ASSERT(rtTimerHandle, "Error Occurred, Timer not created");
    timerState = UNINITIALIZED;
}

/**
 * @brief Default de-constructor makes a timer that can only be polled for state
 * @return Prints a success message if the timer is successfully deleted and a warning message if it was not deleted
*/
Timer::~Timer()
{
    if (xTimerDelete(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD*2) == pdPASS) {
        SOAR_PRINT("Timer has been deleted \n\n");
    }
    else {
        SOAR_PRINT("WARNING, FAILED TO DELETE TIMER! \n\n");
    }
}

/**
 * @brief Callback function for timers.
 * ! MUST be used for ALL timers. Has to be called manually by the user for callback enabled timers. Visit constructor for instructions.
 * @return Sets timer state to COMPLETE when the timer has expired
*/
void Timer::DefaultCallback(TimerHandle_t xTimer){
    Timer* ptrTimer = (Timer*)pvTimerGetTimerID(xTimer);
    ptrTimer->timerState = COMPLETE;
}

/**
 * @brief Changes timer period, Sets timer state back to uninitialized and stops timer
 * @return Returns true if the period is successfully changed and stopped and returns false otherwise
 */
bool Timer::ChangePeriodMs(const uint32_t period_ms)
{
    if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
        if (xTimerStop(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
            timerPeriod = period_ms;
            timerState = UNINITIALIZED;
            return true;
        }
    }
    return false;
}

/**
 * @brief Changes timer period, Sets timer state back to counting and starts timer
 * @return Returns true if the period is successfully changed and returns false otherwise
 */
bool Timer::ChangePeriodMsAndStart(const uint32_t period_ms)
{
    if (xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(period_ms), DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdTRUE) {
        timerPeriod = period_ms;
        timerState = COUNTING;
        return true;
    }
    return false;
}

/**
 * @brief Starts the timer
 * @return Returns true if timer has successfully started, otherwise returns false
*/
bool Timer::Start()
{
    // Return in COMPLETE and COUNTING as it is not possible to start in those states
    if ((timerState == COMPLETE) || (timerState == COUNTING)) {
        return false;
    }
    // Changes timer period to time left when it was previously stopped
    else if (timerState == PAUSED) {
        xTimerChangePeriod(rtTimerHandle, MS_TO_TICKS(remainingTimeBetweenPauses), DEFAULT_TIMER_COMMAND_WAIT_PERIOD);
        timerState = COUNTING;
        return true;
    }
    if (xTimerStart(rtTimerHandle, DEFAULT_TIMER_COMMAND_WAIT_PERIOD) == pdPASS) {
        timerState = COUNTING;
        return true;
    }
    return false;
}

/**
 * @brief Stops the timer
 * @return Returns true if timer has successfully stopped, otherwise returns false
*/
bool Timer::Stop()
{
    // Checks if timer is in counting state because it cannot be stopped in any other state
    if (timerState != COUNTING) {
        return false;
    }
    // Calculates the time left on the timer before it is paused
    remainingTimeBetweenPauses = GetRTOSTimeRemaining();
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
        SOAR_PRINT("Cannot Restart as timer has not yet started!");
        return false;
    }
    if (ChangePeriodMs(timerPeriod) == true) {
        return true;
    }

    return false;
}

/**
 * @brief Restarts Timer and starts counting
*/
bool Timer::ResetTimerAndStart()
{
    if (timerState == UNINITIALIZED) {
        SOAR_PRINT("Cannot Restart as timer has not yet started!");
        return false;
    }
    if (ChangePeriodMsAndStart(timerPeriod) == true) {
        return true;
    }
    return false;
}

/**
 * @param Sets timer to auto-reload if parameter is set to true, Sets timer to one shot if parameter is set to false
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
 * @return Returns true if the timer is set to autoreload and false if it is set to one-shot
*/
const bool Timer::GetIfAutoReload()
{
    return uxTimerGetReloadMode(rtTimerHandle)) == (( UBaseType_t ) pdTRUE)
}

/**
 * @brief Returns timer state enum
 * @return Returns the current state the timer is in
*/
const TimerState Timer::GetState()
{
    return timerState;
}

/**
 * @return Returns the timers' period in milliseconds (ms)
*/
const uint32_t Timer::GetPeriodMs()
{
    return (TICKS_TO_MS(xTimerGetPeriod(rtTimerHandle)));
}

/**
 * @return Returns remaining time (in milliseconds) on timer based on current state
*/
const uint32_t Timer::GetRemainingTimeMs()
{
    if (timerState == UNINITIALIZED){
        return (GetPeriodMs());
        }
    else if (timerState == COUNTING){
        return GetRTOSTimeRemaining();
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
 * @return Returns remaining time in milliseconds
 */
const uint32_t Timer::GetRTOSTimeRemaining()
{
    return (TICKS_TO_MS(xTimerGetExpiryTime(rtTimerHandle) - xTaskGetTickCount()));;
}
