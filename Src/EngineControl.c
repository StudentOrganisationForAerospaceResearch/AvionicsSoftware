#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"
#include "FlightPhase.h"
#include "Data.h"
#include "ValveControl.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int BURN_DURATION = 11000;
static const int POST_BURN_PERIOD = 1000;

static const int POST_BURN_REOPEN_INJECTION_VALVE_DURATION = 10 * 60 * 1000; // 10 minutes
static const int MAX_TANK_PRESSURE = 820000; // 820 psi, 5660 kPa, 25 deg C at saturation
static const int PRELAUNCH_VENT_PULSE_DURATION = 2 * 1000; // 2 seconds

int8_t oxidizerTankIsOverPressure(OxidizerTankPressureData* data)
{
    int32_t tankPressure = -1;

    if (osMutexWait(data->mutex_, 0) == osOK)
    {
        // read tank pressure
        tankPressure = data->pressure_;
        osMutexRelease(data->mutex_);
    }

    if (tankPressure > MAX_TANK_PRESSURE)
    {
        return 1;
    }

    return 0;
}

/**
 * This routine keeps the injection valve closed during prelaunch.
 * This routine exits when the current flight phase is no longer PRELAUNCH.
 */
void engineControlPrelaunchRoutine(OxidizerTankPressureData* data)
{
    uint32_t prevWakeTime = osKernelSysTick();

    // If 0, no venting
    // If non 0, vent for that much more time
    int32_t ventPulseCounter = 0;
    closeInjectionValve();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);

        closeInjectionValve();

        // handle request
        if (pulseVentValveRequested)
        {
            ventPulseCounter = PRELAUNCH_VENT_PULSE_DURATION;
            pulseVentValveRequested = 0; // request has been dealt with
        }

        // if not already venting, check if over pressure
        if (ventPulseCounter <= 0)
        {
            closeVentValve(); // close vent valve if not venting

            if (oxidizerTankIsOverPressure(data))
            {
                ventPulseCounter = PRELAUNCH_VENT_PULSE_DURATION;
            }
        }
        else
        {
            // venting is requested, continue venting process
            openVentValve();
            ventPulseCounter -= PRELAUNCH_PHASE_PERIOD;
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
    closeVentValve();
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
            openInjectionValve();
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
