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
#include "FreeRTOS.h"

/* Macros --------------------------------------------------------------------*/
constexpr uint32_t DEFAULT_TIMER_COMMAND_WAIT_PERIOD = MS_TO_TICKS(15); // Default time to block a task if a command cannot be issued to the timer

#define DEFAULT_TIMER_PERIOD (MS_TO_TICKS(1000)) // 1s

// Enumeration representing the 4 timer states
enum TimerState {
	UNINITIALIZED=0,
	COUNTING,
	PAUSED,
	COMPLETE
};


/* Class -----------------------------------------------------------------*/

/**
 * @brief Timer class
 *
 * Wrapper for FreeRTOS Timers
*/
class Timer
{
public:
	Timer(); // Default Constructor (Polling Timer)
	Timer(void (*TimerCallbackFunction_t)( TimerHandle_t xTimer )); // Constructor for Callback Enabled Timer
	~Timer();
	bool ChangePeriod(const uint32_t period_ms);
	bool ChangePeriodAndStart(const uint32_t period_ms);
	bool Start();
	bool Stop();
	bool ResetTimer();
	bool ResetTimerAndStart();
	void SetAutoReload(bool setReloadOn);

	bool GetAutoReload();
	TimerState GetState();
	uint32_t GetPeriod();
	uint32_t GetRemainingTime();
	uint32_t rtosTimeRemaning();
	static void CallbackFunction( TimerHandle_t xTimer );

protected:

	TimerHandle_t rtTimerHandle;
	TimerState timerState;
	uint32_t remainingTime;
	uint32_t remainingTimeBetweenPauses;
};



#endif /* AVIONICS_INCLUDE_SOAR_CORE_TIMER_H*/
