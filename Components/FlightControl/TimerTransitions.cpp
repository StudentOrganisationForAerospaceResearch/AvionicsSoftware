/**
 ******************************************************************************
 * File Name          : TimerTransitions.cpp
 * Description        : Primary Watchdog task, default task for the system.
 ******************************************************************************
*/
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "TimerTransitions.hpp"
#include "FlightTask.hpp"



void TimerTransitions::EnterIgnition() {
    SOAR_PRINT("Entering IGNITION state...\n");
    ignitionCountdown = Timer(IngnitionToLaunchCallback);
    ignitionCountdown.ChangePeriodMsAndStart(10000);
    return;
}

void TimerTransitions::IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle) {
//    SOAR_PRINT("Changing State to LAUNCH....\n");

    if ((Inst().arrLanuchConfirmFlags == true)) {
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_IGNITION_TO_LAUNCH));
    }
    else {
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_GOTO_ARM));
    }

    Timer::DefaultCallback(rtTimerHandle);
    return;
}


