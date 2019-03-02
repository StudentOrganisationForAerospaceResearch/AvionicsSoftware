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