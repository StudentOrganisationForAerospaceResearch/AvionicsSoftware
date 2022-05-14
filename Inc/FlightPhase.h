#pragma once

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern uint8_t groundSystemsRxChar;
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

static const int FLIGHT_PHASE_CHECK_PERIOD = 1000;
static const uint8_t LAUNCH_CMD_BYTE = 0x20;
static const uint8_t ARM_CMD_BYTE = 0x21;
static const uint8_t ABORT_CMD_BYTE = 0x2F;
static const uint8_t RESET_AVIONICS_CMD_BYTE = 0x4F;
static const uint8_t HEARTBEAT_BYTE = 0x46;
static const uint8_t OPEN_INJECTION_VALVE = 0x2A;
static const uint8_t CLOSE_INJECTION_VALVE = 0x2B;


void newFlightPhase(FlightPhase newPhase);
FlightPhase getCurrentFlightPhase();
void resetFlightPhase();
void flightPhaseTask(void const* flightPhaseQueue);

