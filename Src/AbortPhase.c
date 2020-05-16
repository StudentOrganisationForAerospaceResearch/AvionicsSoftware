#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AbortPhase.h"
#include "FlightPhase.h"
#include "ValveControl.h"

static const int ABORT_PHASE_TASK_PERIOD = 100;

void abortPhaseTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, ABORT_PHASE_TASK_PERIOD);

        if (!IS_ABORT_PHASE)
        {
            // Do nothing if not in abort
            resetAvionicsCmdReceived = 0;
        }
        else
        {
            openLowerVentValve();

            // Detect a reset
            if (resetAvionicsCmdReceived)
            {
                // Reset global variables
                closeLowerVentValve();
                launchCmdReceived = 0;
                abortCmdReceived = 0;
                resetAvionicsCmdReceived = 0;
                heartbeatTimer = HEARTBEAT_TIMEOUT;
                resetFlightPhase();
            }
        }
    }
}
