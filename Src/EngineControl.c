#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"
#include "FlightPhase.h"
#include "Data.h"
#include "ValveControl.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int BURN_DURATION = 12000;
static const int POST_BURN_PERIOD = 1000;

static const int POST_BURN_REOPEN_INJECTION_VALVE_DURATION = 10 * 60 * 1000; // 10 minutes

/**
 * This routine keeps the injection valve closed during prelaunch.
 * This routine exits when the current flight phase is no longer PRELAUNCH.
 */
void engineControlPrelaunchRoutine(OxidizerTankPressureData* data)
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t pulseVentCounter = 0;

    closeInjectionValve();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);

        closeInjectionValve();

        int32_t oxidizerTankPressure = -1;

        // Pulse vent valve
        pulseVentCounter += PRELAUNCH_PHASE_PERIOD;

        if (osMutexWait(data->mutex_, 0) == osOK)
        {
            // read tank pressure
        	oxidizerTankPressure = data->pressure_;
            osMutexRelease(data->mutex_);

            if(oxidizerTankPressure >= 825)
            {
                if (pulseVentCounter <= PRELAUNCH_VALVE_PULSE_MAX_TIME)
                {
                    openUpperVentValve();
                }
                else if (pulseVentCounter <= 2*PRELAUNCH_VALVE_PULSE_MAX_TIME)
                {
                    closeUpperVentValve();
                }
            }
            else if(oxidizerTankPressure >= 850)
            {
                if (pulseVentCounter <= PRELAUNCH_VALVE_PULSE_MAX_TIME)
                {
                    openLowerVentValve();
                }
                else if (pulseVentCounter <= 2*PRELAUNCH_VALVE_PULSE_MAX_TIME)
                {
                    closeLowerVentValve();
                }
            }
            else if(oxidizerTankPressure >= 875)
            {
                if (pulseVentCounter <= PRELAUNCH_VALVE_PULSE_MAX_TIME)
                {
                    openUpperVentValve();
                    openLowerVentValve();
                }
                else if (pulseVentCounter <= 2*PRELAUNCH_VALVE_PULSE_MAX_TIME)
                {
                    closeUpperVentValve();
                    closeLowerVentValve();
                }
            }
            else if(oxidizerTankPressure >= 950)
            {
                newFlightPhase(ABORT);
            }
            else
            {
                pulseVentCounter = 0;
            }
        }
        // If pressure mutex is already in use, we still need to make sure the valves are not open
        // for more than PRELAUNCH_VALVE_PULSE_MAX_TIME
        else if(pulseVentCounter >= PRELAUNCH_VALVE_PULSE_MAX_TIME)
        {
            closeUpperVentValve();
            closeLowerVentValve();
            pulseVentCounter = 0;
        }

        if (launchCmdReceived >= 2 && systemIsArmed)
        {
            newFlightPhase(BURN);
        }

        if (getCurrentFlightPhase() != PRELAUNCH)
        {
            return;
        }
    }
}

/**
 * This routine opens the injection valve for the burn phase
 * for a preconfigured amount of time. Once the preconfigured amount
 * of time has passed, this routine updates the current flight phase.
 */
void engineControlBurnRoutine()
{
    closeUpperVentValve();
    openInjectionValve();
    osDelay(BURN_DURATION);
    newFlightPhase(COAST);
    return;
}

/**
 * This routine is the final phase.
 */
void engineControlPostBurnRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInPostBurn = 0;
    uint32_t pulseVentCounter = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, POST_BURN_PERIOD);
        FlightPhase phase = getCurrentFlightPhase();

        if (phase != COAST && phase != DROGUE_DESCENT && phase != MAIN_DESCENT)
        {
            return;
        }

        // requires 49 days to overflow, not handling this case
        timeInPostBurn += POST_BURN_PERIOD;

        if (timeInPostBurn < POST_BURN_REOPEN_INJECTION_VALVE_DURATION)
        {
            closeInjectionValve();
        }
        else
        {
            pulseVentCounter += POST_BURN_PERIOD;

            if (pulseVentCounter <= POST_BURN_VALVE_PULSE_MAX_TIME)
            {
                openUpperVentValve();
                openLowerVentValve();
            }
            else if (pulseVentCounter <= 2*POST_BURN_VALVE_PULSE_MAX_TIME)
            {
                closeUpperVentValve();
                closeLowerVentValve();
            }
            else
            {
                pulseVentCounter = 0;
            }            
        }
    }
}

void engineControlTask(void const* arg)
{
    OxidizerTankPressureData* data = (OxidizerTankPressureData*) arg;

    for (;;)
    {
        switch (getCurrentFlightPhase())
        {
            case PRELAUNCH:
                engineControlPrelaunchRoutine(data);
                break;

            case BURN:
                engineControlBurnRoutine();
                break;

            case COAST:  // fall through
            case DROGUE_DESCENT:
            case MAIN_DESCENT:
                engineControlPostBurnRoutine();
                break;

            case ABORT:

                // Do nothing and let other code do what needs to be done
                osDelay(PRELAUNCH_PHASE_PERIOD);

                break;

            default:
                break;
        }
    }
}
