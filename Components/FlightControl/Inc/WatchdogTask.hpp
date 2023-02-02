/**
 ******************************************************************************
 * File Name          : FlightTask.hpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_FLIGHTTASK_HPP_
#define SOAR_FLIGHTTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "Timer.hpp"

class WatchdogTask : public Task
{
public:
	static WatchdogTask& Inst() {
		static WatchdogTask inst;
		return inst;
	}
	WatchdogTask();
	WatchdogTask(int timerPeriodMs);		// Constructor that creates a heartbeat with specified time period

protected:
    static void RunTask(void* pvParams) { WatchdogTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code


private:
    // Private Functions
    WatchdogTask();        // Private constructor
    WatchdogTask(const WatchdogTask&);                        // Prevent copy-construction
    WatchdogTask& operator=(const WatchdogTask&);            // Prevent assignment

    static void HeartbeatFailureCallback(TimerHandle_t rtTimerHandle);						// Callback for timer which aborts system in case of data ghosting
    static Timer HeartbeatPeriod(void (*Callback)());				// Timer that resets system if triggered
    void RecieveHeartbeat();										// If it recieves a heartbeat then it resets the timer

};

#endif    // SOAR_FLIGHTTASK_HPP_
