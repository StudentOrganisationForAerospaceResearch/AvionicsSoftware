#pragma once

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

osMutexId flightPhaseMutex; // ONLY PUBLIC FOR INITIALIZATION PURPOSES

typedef enum
{
    PRELAUNCH,
    BURN,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    ABORT_RECEIVED_COMMAND,
    ABORT_COMMUNICATION_ERROR,
    ABORT_OXIDIZER_PRESSURE,
    ABORT_UNSPECIFIED_REASON
} FlightPhase;

void newFlightPhase(FlightPhase newPhase);
FlightPhase getCurrentFlightPhase();
void resetFlightPhase();

