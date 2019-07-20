#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"
#include "FlightPhase.h"
#include "Data.h"
#include "ValveControl.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int BURN_DURATION = 8500;
static const int POST_BURN_PERIOD = 1000;

static const int POST_BURN_REOPEN_LOWER_VENT_VALVE_DURATION = 10 * 60 * 1000; // 10 minutes

/**
 * This routine keeps the injection valve closed during prelaunch.
 * This routine exits when the current flight phase is no longer PRELAUNCH.
 */
void engineControlPrelaunchRoutine(OxidizerTankPressureData* data)
{
    uint32_t prevWakeTime = osKernelSysTick();

    closeInjectionValve();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);

        closeInjectionValve();
        closeLowerVentValve();

        int32_t oxidizerTankPressure = -1;

        // if (osMutexWait(data->mutex_, 0) == osOK)
        // {
        //     // read tank pressure
        //     oxidizerTankPressure = data->pressure_;
        //     osMutexRelease(data->mutex_);

        //     if (oxidizerTankPressure >= 850 * 1000)
        //     {
        //         newFlightPhase(ABORT_OXIDIZER_PRESSURE);
        //     }
        // }

        if (launchCmdReceived >= 2 && ARM == getCurrentFlightPhase())
        {
            newFlightPhase(BURN);
        }

        if (PRELAUNCH != getCurrentFlightPhase() && ARM != getCurrentFlightPhase())
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
    closeLowerVentValve();
    openInjectionValve();
    osDelay(BURN_DURATION);
    newFlightPhase(COAST);
    return;
}

/**
 * This routine closes all valves and changes to the PostFlightRoutine
 * after 10 mins passed since COAST started.
 */
void engineControlPostBurnRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInPostBurn = 0;

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

        if (timeInPostBurn < POST_BURN_REOPEN_LOWER_VENT_VALVE_DURATION)
        {
            closeInjectionValve();
            closeLowerVentValve();
        }
        else
        {
            newFlightPhase(POST_FLIGHT);
        }
    }
}

/**
 * This routine is the final phase.
 */
void engineControlPostFlightRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInPostBurn = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, POST_BURN_PERIOD);
        FlightPhase phase = getCurrentFlightPhase();

        if (phase != POST_FLIGHT)
        {
            return;
        }

        closeInjectionValve();
        openLowerVentValve();
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
            case ARM:
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

            case POST_FLIGHT:
                engineControlPostFlightRoutine();
                break;

            // All aborts fall through here because they all do the same thing in each case
            case ABORT_COMMAND_RECEIVED:
            case ABORT_OXIDIZER_PRESSURE:
            case ABORT_UNSPECIFIED_REASON:
            case ABORT_COMMUNICATION_ERROR:

                // Do nothing and let other code do what needs to be done
                osDelay(PRELAUNCH_PHASE_PERIOD);

                break;

            default:
                break;
        }
    }
}
