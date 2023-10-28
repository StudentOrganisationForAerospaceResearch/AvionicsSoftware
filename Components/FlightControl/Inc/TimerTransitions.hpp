/**
 ******************************************************************************
 * File Name          : TimerTransitions.hpp
 * Description        : Handles Automatic Ignition Sequence and following timed transitions
 ******************************************************************************
*/
#ifndef SOAR_TIMERTRANSITIONS_HPP_
#define SOAR_TIMERTRANSITIONS_HPP_
#include "FlightTask.hpp"
#include "RocketSM.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint32_t IGINITION_TIMER_PERIOD_MS = 20000;
//constexpr uint32_t IR_IGINITION_TIMER_PERIOD = 8000;

constexpr uint32_t BURN_TIMER_PERIOD_MS =
    2 *
    1000;  // LAUNCH -> BURN Timer is      2 seconds (LAUNCH for 2 seconds after confirmation)
constexpr uint32_t COAST_TIMER_PERIOD_MS =
    5 * 1000;  // BURN -> COAST Timer is       5 seconds (BURN for 5 seconds)
constexpr uint32_t DESCENT_TIMER_PERIOD_MS =
    2 * 60 *
    1000;  // COAST -> DESCENT Timer is    2 minutes (COAST for 2 minutes) -- Shorter, but since vents open in DESCENT this is longer for safety
constexpr uint32_t RECOVERY_TIMER_PERIOD_MS =
    10 *
    1000;  // DESCENT -> RECOVERY Timer is 10 seconds(DESCENT for 10 seconds) -- Vents Open

class TimerTransitions {
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
        FlightTask::Inst().SendCommand(
            Command(CONTROL_ACTION, RSC_LAUNCH_TO_BURN));
    }
    inline static void BurnToCoastCallback(TimerHandle_t rtTimerHandle) {
        FlightTask::Inst().SendCommand(
            Command(CONTROL_ACTION, RSC_BURN_TO_COAST));
    }
    inline static void CoastToDescentCallback(TimerHandle_t rtTimerHandle) {
        FlightTask::Inst().SendCommand(
            Command(CONTROL_ACTION, RSC_COAST_TO_DESCENT));
    }
    inline static void DescentToRecoveryCallback(TimerHandle_t rtTimerHandle) {
        FlightTask::Inst().SendCommand(
            Command(CONTROL_ACTION, RSC_DESCENT_TO_RECOVERY));
    }

   private:
    Timer* ignitionCountdown;
    Timer* burnCountdown;
    Timer* coastCountdown;
    Timer* descentCountdown;
    Timer* recoveryCountdown;
};

#endif  // SOAR_TIMERTRANSITIONS_HPP_
