/**
 ******************************************************************************
 * File Name          : FlightTask.hpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_WATCHDOGTASK_HPP_
#define SOAR_WATCHDOGTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "Timer.hpp"
    
/* Macros/Enums ------------------------------------------------------------*/
enum RADIOHB_COMMANDS {
    RADIOHB_NONE = 0,
    RADIOHB_REQUEST,
    RADIOHB_DISABLED
};

class WatchdogTask : public Task
{
public:
    static WatchdogTask& Inst() {
        static WatchdogTask inst;
        return inst;
    }

    void InitTask();


protected:
    static void RunTask(void* pvParams) { WatchdogTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code


private:
    // Private Functions
    WatchdogTask();        // Private constructor
    WatchdogTask(const WatchdogTask&);                        // Prevent copy-construction
    WatchdogTask& operator=(const WatchdogTask&);            // Prevent assignment

    static void HeartbeatFailureCallback(TimerHandle_t rtTimerHandle);    // Callback for timer which aborts system in case of data ghosting
    void HandleCommand(Command& cm);
    void HandleHeartbeat(uint16_t taskCommand);                        // If it receives a heartbeat then it resets the timer
    Timer heartbeatTimer;


};

#endif    // SOAR_WATCHDOGTASK_HPP_
