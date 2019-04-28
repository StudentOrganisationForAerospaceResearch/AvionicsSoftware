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
    uint32_t pulseVentCounter = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, ABORT_PHASE_TASK_PERIOD);

        if (getCurrentFlightPhase() != ABORT)
        {
            // Do nothing if not in abort
            resetAvionicsCmdReceived = 0;
        }
        else
        {
            // Pulse vent valve
            pulseVentCounter += ABORT_PHASE_TASK_PERIOD;

            if (pulseVentCounter <= ABORT_VALVE_PULSE_MAX_TIME)
            {
                openUpperVentValve();
                openLowerVentValve();
            }
            else if (pulseVentCounter <= 2*ABORT_VALVE_PULSE_MAX_TIME)
            {
                closeUpperVentValve();
                closeLowerVentValve();
            }
            else
            {
                pulseVentCounter = 0;
            }

            // Detect a reset
            if (resetAvionicsCmdReceived)
            {
                // Reset local variables
                pulseVentCounter = 0;

                // Reset global variables
                closeUpperVentValve();
                closeLowerVentValve();
                launchCmdReceived = 0;
                systemIsArmed = 0;
                abortCmdReceived = 0;
                resetAvionicsCmdReceived = 0;
                heartbeatTimer = HEARTBEAT_TIMEOUT;
                resetFlightPhase();
            }
        }
    }
}
