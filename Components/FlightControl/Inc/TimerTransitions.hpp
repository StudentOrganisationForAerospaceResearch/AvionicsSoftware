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

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint32_t IGINITION_TIMER_PERIOD = 20000;
//constexpr uint32_t IR_IGINITION_TIMER_PERIOD = 8000;
constexpr uint32_t RECOVERY_TIMER_PERIOD = 30000;

class TimerTransitions
{
public:
    static TimerTransitions& Inst() {
        static TimerTransitions inst;
        return inst;
    }
    TimerTransitions();
    void IgnitionSequence();
    void IRSequence();
    bool ignitionConformation = false;
    void ManualLaunch();
    void Setup();

protected:
    static void IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle);

private:
    Timer* ignitionCountdown;
    Timer* recoveryTimers;
};

#endif    // SOAR_TIMERTRANSITIONS_HPP_
