#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AbortPhase.h"
#include "FlightPhase.h"
#include "ValveControl.h"

static const int ABORT_PHASE_TASK_PERIOD = 50;
static const int ABORT_INJECTION_DELAY = 5 * 60 * 1000; // 5 minutes

void abortPhaseTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInAbort = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, ABORT_PHASE_TASK_PERIOD);

        if (getCurrentFlightPhase() == ABORT)
        {
            // close injection valve
            closeInjectionValve();

            // pulse vent valve
            for (;;)
            {
                openVentValve();
                osDelay(MAX_DURATION_VENT_VALVE_OPEN);
                timeInAbort += MAX_DURATION_VENT_VALVE_OPEN;

                closeVentValve();
                osDelay(REQUIRED_DURATION_VENT_VALVE_CLOSED);
                timeInAbort += REQUIRED_DURATION_VENT_VALVE_CLOSED;

                if (timeInAbort > ABORT_INJECTION_DELAY)
                {
                    openInjectionValve();
                }
                else
                {
                    closeInjectionValve();
                }

                if (resetAvionicsCmdReceived)
                {
                    launchCmdReceived = 0;
                    abortCmdReceived = 0;

                    timeInAbort = 0;

                    resetFlightPhase();
                    break;
                }
            }
        }
    }
}
