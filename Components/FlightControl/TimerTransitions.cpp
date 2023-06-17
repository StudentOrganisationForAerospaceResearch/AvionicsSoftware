/**
 ******************************************************************************
 * File Name          : TimerTransitions.cpp
 * Description        : Handles Automatic Ignition Sequence and following timed transitions
 ******************************************************************************
*/
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "TimerTransitions.hpp"
#include "FlightTask.hpp"

TimerTransitions::TimerTransitions() {
//    ignitionConformation = nullptr;
}

void TimerTransitions::Setup() {
	ignitionCountdown = new Timer(IngnitionToLaunchCallback);
	ignitionCountdown->ChangePeriodMs(IGINITION_TIMER_PERIOD);
}

void TimerTransitions::IgnitionSequence() {
//    SOAR_PRINT("Entering IGNITION state...\n");
    ignitionCountdown->Start();
    return;
}

//void TimerTransitions::IRSequence() {
//    ignitionCountdown->ChangePeriodMsAndStart(IR_IGINITION_TIMER_PERIOD);
//    ignitionConformation = true;
//    return;
//}

void TimerTransitions::ManualLaunch() {
	ignitionConformation = true;
	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_IGNITION_TO_LAUNCH));
}

void TimerTransitions::IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle) {
//    SOAR_PRINT("Changing State to LAUNCH....\n");

    if ((Inst().ignitionConformation == true)) {
//        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_IGNITION_TO_LAUNCH));
        Timer::DefaultCallback(rtTimerHandle);
    }
    else {
    	Inst().ignitionCountdown->ResetTimer();
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_GOTO_ARM));
    }

    return;
}


