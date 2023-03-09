/**
 ******************************************************************************
 * File Name          : TimerTransitions.cpp
 * Description        : Primary Watchdog task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_TIMERTRANSITIONS_HPP_
#define SOAR_TIMERTRANSITIONS_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "Timer.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum TIMERTRANSITION_CONTROLS  {
    CONFIRM_LAUNCH_2 = 0,
};

class TimerTransitions
{
public:
	static TimerTransitions& Inst() {
		static TimerTransitions inst;
		return inst;
	}
	void EnterIgnition();
	void ExitLaunch();
	void BurnSequence();
	void InitiateBurn();
	void CheckBurnSequence();
	bool arrLanuchConfirmFlags = false;

protected:
	static void IngnitionToLaunchCallback(TimerHandle_t rtTimerHandle);
	static void BurnToCoastCallback(TimerHandle_t rtTimerHandle);

private:
	Timer ignitionCountdown;
	Timer burnCountdown;
};

#endif    // SOAR_TIMERTRANSITIONS_HPP_
