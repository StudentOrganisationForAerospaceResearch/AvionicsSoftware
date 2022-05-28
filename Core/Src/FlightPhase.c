#include "FlightPhase.h"

static FlightPhase currentFlightPhase = PRELAUNCH;

static const int FLIGHT_MUTEX_RETRIES = 10;

void newFlightPhase(FlightPhase newPhase)
{
    if (newPhase <= getCurrentFlightPhase())
    {
        return;
    }

    for (int i = 0; i < FLIGHT_MUTEX_RETRIES; i++)
    {
        if (osMutexWait(flightPhaseMutex, 0) == osOK)
        {
            if (newPhase > currentFlightPhase)
            {
                currentFlightPhase = newPhase;
            }

            osMutexRelease(flightPhaseMutex);
            return;
        }
    }

    // Hopefully it never gets here
    // if it does, risk a race condition
    // better than not setting the phase
    currentFlightPhase = newPhase;
    return;
}

FlightPhase getCurrentFlightPhase()
{
    FlightPhase phase;

    for (int i = 0; i < FLIGHT_MUTEX_RETRIES; i++)
    {
        if (osMutexWait(flightPhaseMutex, 0) == osOK)
        {
            phase = currentFlightPhase;
            osMutexRelease(flightPhaseMutex);
            return phase;
        }
    }

    // Hopefully it never gets here
    // if it does, risk a race condition
    // better than returning invalid phase
    return currentFlightPhase;
}

void resetFlightPhase()
{
    for (int i = 0; i < FLIGHT_MUTEX_RETRIES; i++)
    {
        if (osMutexWait(flightPhaseMutex, 0) == osOK)
        {
            currentFlightPhase = PRELAUNCH;
            osMutexRelease(flightPhaseMutex);
            return;
        }
    }

    // Hopefully it never gets here
    // if it does, risk a race condition
    // better than not setting the phase
    currentFlightPhase = PRELAUNCH;
    return;
}

int isAbortPhase()
{
	FlightPhase currPhase = getCurrentFlightPhase();
	if(currPhase == ABORT_COMMAND_RECEIVED ||
			currPhase == ABORT_NO_HEARTBEAT ||
			currPhase == ABORT_COMMUNICATION_ERROR ||
			currPhase == ABORT_OXIDIZER_PRESSURE ||
			currPhase == ABORT_UNSPECIFIED_REASON
			)
	{
		return 1;
	}
	return 0;
}

void gsListenerTask(void const* arg)
{
	HAL_UART_Receive_IT(&GS_UART, &groundSystemsRxChar, 1);
}

void flightPhaseTask(void const* flightPhaseQueue)
{
	uint8_t cmd = 0;
	int heartbeatTimer = HEARTBEAT_COUNTDOWN;
	int launchCounter = 0;
	int burnTimer = BURN_TO_COST_TIME;
	int coastTimer = COAST_TO_POST_TIME;
	for(;;)
	{
		uint32_t prevWakeTime = osKernelSysTick();
		FlightPhase currPhase = getCurrentFlightPhase();
		/*
		 * Uncomment section to test with Debug UART
		 */
//		if (xQueueReceive(flightPhaseQueue, &cmd, 0) != pdTRUE)
//		{
//			HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)"Error in Receiving from Queue\n\n", 31, 1000);
//		}
//		else
//		{
//			HAL_UART_Transmit(&DEBUG_UART, &cmd, sizeof(uint8_t), 1000);
//		}
		// Check heart beat, if no heart beat, go into abort
		if(heartbeatTimer <= 0 && currPhase != ABORT_NO_HEARTBEAT)
		{
			newFlightPhase(ABORT_NO_HEARTBEAT);
		}
		// If in burn state, decrement burn timer, switch to coast if timer hits 0
		else if(currPhase == BURN)
		{
			burnTimer -= FLIGHT_PHASE_CHECK_PERIOD;
			if(burnTimer <= 0)
			{
				newFlightPhase(COAST);
			}
		}
		// If in coast state, decrement coast timer, switch to post flight if timer hits 0
		else if(currPhase == COAST)
		{
			coastTimer -= FLIGHT_PHASE_CHECK_PERIOD;
			if(coastTimer <= 0)
			{
				newFlightPhase(POST_FLIGHT);
			}
		}
		// Else we're on the ground and should carry out commands from the flightphase queue
		else
		{
			// Attempt to get next command in queue, if queue is empty or error, return pdFalse with 0 delay
			// Thus, this if block will only execute if a command is pulled out of the queue
			if(xQueueReceive(flightPhaseQueue, &cmd, 0) == pdTRUE)
			{
				switch(cmd)
				{
					case CLEAR_FLASH_CMD_BYTE:
					{
						if(currPhase == PRELAUNCH)
						{
							// TODO: clear flash memory, either directly using SPI or setting a flag for logdata task
						}
						break;
					}
					case LAUNCH_CMD_BYTE:
					{
						if(currPhase == ARM)
						{
							launchCounter++;
						}
						if(launchCounter >= LAUNCH_COUNT)
						{
							newFlightPhase(BURN);
						}
						break;
					}
					case ARM_CMD_BYTE:
					{
						if(currPhase == PRELAUNCH)
						{
							newFlightPhase(ARM);
						}
						break;
					}
					case ABORT_CMD_BYTE:
					{
						newFlightPhase(ABORT_COMMAND_RECEIVED);
						break;
					}
					case RESET_CMD_BYTE:
					{
						if(currPhase == ABORT_COMMAND_RECEIVED || currPhase == ABORT_NO_HEARTBEAT)
						{
							resetFlightPhase();
							heartbeatTimer = HEARTBEAT_COUNTDOWN;
							launchCounter = 0;
							burnTimer = BURN_TO_COST_TIME;
							coastTimer = COAST_TO_POST_TIME;
						}
						break;
					}
					case HEARTBEAT_BYTE:
					{
						heartbeatTimer = HEARTBEAT_COUNTDOWN;
						break;
					}
					default:
					{
						break;
					}
				}
			}
			heartbeatTimer -= FLIGHT_PHASE_CHECK_PERIOD;
		}
		osDelayUntil(&prevWakeTime, FLIGHT_PHASE_CHECK_PERIOD);
	}
}
