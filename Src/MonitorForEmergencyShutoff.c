#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForEmergencyShutoff.h"
#include "FlightPhase.h"
#include "Data.h"
#include "ValveControl.h"

static const int MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD = 1000;

void monitorForEmergencyShutoffTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();

    // AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    FlightPhase phase = PRELAUNCH;
    // int32_t magnetoZ = -1;

    heartbeatTimer = HEARTBEAT_TIMEOUT; // Timer counts down to 0

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD);

        phase = getCurrentFlightPhase();

        // if (osMutexWait(data->mutex_, 0) == osOK)
        // {
        //     magnetoZ = data->magnetoZ_;
        //     osMutexRelease(data->mutex_);
        // }

        switch (getCurrentFlightPhase())
        {
            // Fallthrough because umbilical is still supposed to be connected during these flight phases.
            // This means that an ABORT command can be received or a communication error could happen.
            case PRELAUNCH:
            case ARM:
                heartbeatTimer -= MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD;

                int prelaunchCheckStatus = prelaunchChecks();

                if (prelaunchCheckStatus == 1)
                {
                    newFlightPhase(ABORT_COMMAND_RECEIVED);
                }
                else if (prelaunchCheckStatus == 2)
                {
                    newFlightPhase(ABORT_COMMUNICATION_ERROR);
                }

                break;

            // Fallthrough because the umbilical should be disconnected during these flight phases.
            // This means that a communication error is not a valid way to get to ABORT. The only
            // way the ABORT command can be received is if the rocket fails to take off and the
            // umbilical does not disconnect.
            case BURN:
            case COAST:
            case DROGUE_DESCENT:
            case MAIN_DESCENT:
            case POST_FLIGHT:
                if (postArmChecks())
                {
                    newFlightPhase(ABORT_COMMAND_RECEIVED);
                }

                // check if not right side up
                // if ()
                // {
                //     newFlightPhase(ABORT);
                // }
                break;

            default:
                // do nothing
                break;
        }
    }
}

// Return 0 for everything ok
int prelaunchChecks()
{
    if (abortCmdReceived)
    {
        return 1;
    }

    // If heartbeatTimer reaches 0, no heartbeat was received for HEARTBEAT_TIMEOUT
    if (heartbeatTimer <= 0)
    {
        return 2;
    }

    return 0;
}

// Return 0 for everything ok
int postArmChecks()
{
    if (abortCmdReceived)
    {
        closeInjectionValve();
        return 1;
    }

    return 0;
}
