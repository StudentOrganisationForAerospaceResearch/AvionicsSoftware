#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForEmergencyShutoff.h"
#include "FlightPhase.h"
#include "Data.h"

static const int MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD = 1000;

void monitorForEmergencyShutoffTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();

    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    FlightPhase phase = PRELAUNCH;
    int32_t magnetoZ = -1;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD);

        phase = getCurrentFlightPhase();

        if (osMutexWait(data->mutex_, 0) == osOK)
        {
            magnetoZ = data->magnetoZ_;
            osMutexRelease(data->mutex_);
        }

        switch (getCurrentFlightPhase())
        {
            case PRELAUNCH:
                if (abortCmdReceived)
                {
                    newFlightPhase(ABORT);
                }

                break;

            case BURN:
                // check if not right side up
                // if ()
                // {
                //     newFlightPhase(ABORT);
                // }
                break;

            case DROGUE_DESCENT:
            case MAIN_DESCENT:
            case ABORT:
            default:
                // do nothing
                break;
        }
    }
}

