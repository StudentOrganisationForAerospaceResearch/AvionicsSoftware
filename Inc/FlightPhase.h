#pragma once

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define IS_ABORT_PHASE ( \
            getCurrentFlightPhase() == ABORT_COMMAND_RECEIVED || \
            getCurrentFlightPhase() == ABORT_COMMUNICATION_ERROR || \
            getCurrentFlightPhase() == ABORT_OXIDIZER_PRESSURE || \
            getCurrentFlightPhase() == ABORT_UNSPECIFIED_REASON \
        )

osMutexId flightPhaseMutex; // ONLY PUBLIC FOR INITIALIZATION PURPOSES

typedef enum
{
    PRELAUNCH,
    ARM,
    BURN,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    POST_FLIGHT,
    ABORT_COMMAND_RECEIVED,
    ABORT_COMMUNICATION_ERROR,
    ABORT_OXIDIZER_PRESSURE,
    ABORT_UNSPECIFIED_REASON
} FlightPhase;

void newFlightPhase(FlightPhase newPhase);
FlightPhase getCurrentFlightPhase();
void resetFlightPhase();

