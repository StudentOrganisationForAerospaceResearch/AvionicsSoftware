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
	// Ignition timer unused
	//ignitionCountdown = new Timer(IngnitionToLaunchCallback);
	//ignitionCountdown->ChangePeriodMs(IGINITION_TIMER_PERIOD_MS);
	burnCountdown = new Timer(LaunchToBurnCallback);
	burnCountdown->ChangePeriodMs(RECOVERY_TIMER_PERIOD_MS);
	coastCountdown = new Timer(BurnToCoastCallback);
	coastCountdown->ChangePeriodMs(RECOVERY_TIMER_PERIOD_MS);
	descentCountdown = new Timer(CoastToDescentCallback);
	descentCountdown->ChangePeriodMs(RECOVERY_TIMER_PERIOD_MS);
	recoveryCountdown = new Timer(DescentToRecoveryCallback);
	recoveryCountdown->ChangePeriodMs(RECOVERY_TIMER_PERIOD_MS);
}

// Ignition sequence unused
//void TimerTransitions::IgnitionSequence() {
////    SOAR_PRINT("Entering IGNITION state...\n");
//    ignitionCountdown->Start();
//    return;
//}

void TimerTransitions::BurnSequence() {
	burnCountdown->Start();
    return;
}

void TimerTransitions::CoastSequence() {
	coastCountdown->Start();
    return;
}

void TimerTransitions::DescentSequence() {
	descentCountdown->Start();
    return;
}

void TimerTransitions::RecoverySequence() {
	recoveryCountdown->Start();
    return;
}

//void TimerTransitions::IRSequence() {
//    ignitionCountdown->ChangePeriodMsAndStart(IR_IGINITION_TIMER_PERIOD);
//    ignitionConformation = true;
//    return;
//}

//void TimerTransitions::ManualLaunch() {
//	ignitionConformation = true;
//	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_IGNITION_TO_LAUNCH));
//}

// Ignition sequence unused
//void TimerTransitions::IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle) {
////    SOAR_PRINT("Changing State to LAUNCH....\n");
//
//    if ((Inst().ignitionConformation == true)) {
////        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_IGNITION_TO_LAUNCH));
//        Timer::DefaultCallback(rtTimerHandle);
//    }
//    else {
//    	Inst().ignitionCountdown->ResetTimer();
//        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_GOTO_ARM));
//    }
//
//    return;
//}

