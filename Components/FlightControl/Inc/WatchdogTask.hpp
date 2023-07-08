/**
 ******************************************************************************
 * File Name          : WatchdogTask.hpp
 * Description        : Primary Watchdog task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_WATCHDOGTASK_HPP_
#define SOAR_WATCHDOGTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "Timer.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum HEARTBEAT_COMMANDS  {
    RADIOHB_NONE = 0,
    RADIOHB_REQUEST,        // Heartbeat countdown timer is reset when HEARTBEAT_COMMAND is sent
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

    static void HeartbeatFailureCallback(TimerHandle_t rtTimerHandle);    // Callback for timer which aborts system in case of data ghosting
    void HandleCommand(Command& cm);
    void HandleHeartbeat(uint16_t taskCommand);                        // If it receives a heartbeat then it resets the timer
    Timer* heartbeatTimer;

private:
    // Private Functions
    WatchdogTask();        // Private constructor
    WatchdogTask(const WatchdogTask&);                        // Prevent copy-construction
    WatchdogTask& operator=(const WatchdogTask&);            // Prevent assignment
};

#endif    // SOAR_WATCHDOGTASK_HPP_
