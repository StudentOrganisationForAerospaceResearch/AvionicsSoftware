#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AbortPhase.h"
#include "FlightPhase.h"
#include "ValveControl.h"

static const int ABORT_PHASE_TASK_PERIOD = 100;
static const int ABORT_INJECTION_DELAY = 5 * 60 * 1000; // 5 minutes

void abortPhaseTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInAbort = 0;
    uint32_t pulseVentCounter = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, ABORT_PHASE_TASK_PERIOD);

        if (getCurrentFlightPhase() != ABORT)
        {
            // don't do anything unless in abort
            resetAvionicsCmdReceived = 0;
            continue;
        }
        else
        {
            timeInAbort += ABORT_PHASE_TASK_PERIOD;

            // open injection valve to quickly empty out oxidizer
            // wait after time period to ensure no recombustion
            if (timeInAbort > ABORT_INJECTION_DELAY)
            {
                openInjectionValve();
            }
            else
            {
                closeInjectionValve();
            }

            // pulse vent valve
            pulseVentCounter += ABORT_PHASE_TASK_PERIOD;

            if (pulseVentCounter < MAX_TIME_VENT_VALVE_OPEN)
            {
                openVentValve();
            }
            else if (pulseVentCounter < MAX_TIME_VENT_VALVE_OPEN + REQUIRED_TIME_VENT_VALVE_CLOSED)
            {
                closeVentValve();
            }
            else
            {
                // pulseVentCounter >  MAX_TIME_VENT_VALVE_OPEN + TIME_VENT_VALVE_CLOSED
                pulseVentCounter = 0;
            }

            // detect a reset
            if (resetAvionicsCmdReceived)
            {
                // reset local variables
                timeInAbort = 0;
                pulseVentCounter = 0;

                // reset global variables
                closeVentValve();
                closeInjectionValve();
                launchCmdReceived = 0;
                systemIsArmed = 0;
                pulseVentValveRequested = 0;
                abortCmdReceived = 0;
                resetAvionicsCmdReceived = 0;
                heartbeatTimer = HEARTBEAT_TIMEOUT;
                resetFlightPhase();
            }
        }
    }
}
