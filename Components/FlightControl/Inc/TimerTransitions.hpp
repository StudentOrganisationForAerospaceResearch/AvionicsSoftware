/**
 ******************************************************************************
 * File Name          : TimerTransitions.hpp
 * Description        : Handles Automatic Ignition Sequence and following timed transitions
 ******************************************************************************
*/
#ifndef SOAR_TIMERTRANSITIONS_HPP_
#define SOAR_TIMERTRANSITIONS_HPP_
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "Timer.hpp"
#include "FlightTask.hpp"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint32_t IGINITION_TIMER_PERIOD_MS = 20000;
//constexpr uint32_t IR_IGINITION_TIMER_PERIOD = 8000;
constexpr uint32_t RECOVERY_TIMER_PERIOD_MS = 30000;

class TimerTransitions
{
public:
    static TimerTransitions& Inst() {
        static TimerTransitions inst;
        return inst;
    }
    TimerTransitions();
    void IgnitionSequence();
    void BurnSequence();
    void CoastSequence();
    void DescentSequence();
    void RecoverySequence();
    void IRSequence();
    bool ignitionConformation = false;
    void ManualLaunch();
    void Setup();

protected:
    static void IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle);
    inline static void LaunchToBurnCallback(TimerHandle_t rtTimerHandle) {
    	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_LAUNCH_TO_BURN));
    }
    inline static void BurnToCoastCallback(TimerHandle_t rtTimerHandle) {
    	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_BURN_TO_COAST));
    }
    inline static void CoastToDescentCallback(TimerHandle_t rtTimerHandle) {
    	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_COAST_TO_DESCENT));
    }
    inline static void DescentToRecoveryCallback(TimerHandle_t rtTimerHandle) {
    	FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_DESCENT_TO_RECOVERY));
    }

private:
    Timer* ignitionCountdown;
    Timer* burnCountdown;
    Timer* coastCountdown;
    Timer* descentCountdown;
    Timer* recoveryCountdown;
};

#endif    // SOAR_TIMERTRANSITIONS_HPP_
