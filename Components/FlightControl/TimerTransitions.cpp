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

TimerTransitions::TimerTransitions() {
	ignitionCountdown = new Timer(IngnitionToLaunchCallback);
	ignitionCountdown->ChangePeriodMs(IGINITION_TIMER_PERIOD);
}

void TimerTransitions::IgnitionSequence() {
//    SOAR_PRINT("Entering IGNITION state...\n");
    ignitionCountdown->Start();
    return;
}

void TimerTransitions::IRSequence() {
	ignitionCountdown->ChangePeriodMsAndStart(IR_IGINITION_TIMER_PERIOD);
	HAL_GPIO_WritePin(ExtLED1_GPIO_Port, ExtLED1_Pin, GPIO_PIN_SET);
	osDelay(300);
	HAL_GPIO_WritePin(ExtLED1_GPIO_Port, ExtLED1_Pin, GPIO_PIN_RESET);
	osDelay(300);
	HAL_GPIO_WritePin(ExtLED1_GPIO_Port, ExtLED1_Pin, GPIO_PIN_SET);
	osDelay(300);
	HAL_GPIO_WritePin(ExtLED1_GPIO_Port, ExtLED1_Pin, GPIO_PIN_RESET);
	osDelay(300);
	HAL_GPIO_WritePin(ExtLED1_GPIO_Port, ExtLED1_Pin, GPIO_PIN_SET);
	ignitionConformation = true;
	return;
}

void TimerTransitions::IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle) {
//    SOAR_PRINT("Changing State to LAUNCH....\n");

    if ((Inst().ignitionConformation == true)) {
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_IGNITION_TO_LAUNCH));
    }
    else {
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_GOTO_ARM));
    }

    Timer::DefaultCallback(rtTimerHandle);
    return;
}


