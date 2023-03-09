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

void TimerTransitions::ExitLaunch() {
	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_LAUNCH_TO_BURN));
	return;
}

void TimerTransitions::InitiateBurn () {
	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_BURN_SEQUENCE));
	return;
}

void TimerTransitions::BurnSequence() {
//	osDelay(100);
	SOAR_PRINT("Burn Started\n");
	burnCountdown = Timer(BurnToCoastCallback);
	burnCountdown.ChangePeriodMsAndStart(7000);
	return;
}

//void TimerTransitions::CheckBurnSequence () {
//	SOAR_PRINT("The time remaining is %d s", burnCountdown.GetRemainingTimeMs());
//}

void TimerTransitions::BurnToCoastCallback(TimerHandle_t rtTimerHandle) {
	SOAR_PRINT("Going to COAST \n");
	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_BURN_TO_COAST));
	return;
}
