#pragma once

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern uint8_t groundSystemsRxChar;
extern osMutexId flightPhaseMutex;

#define CLEAR_FLASH_CMD_BYTE 0x1F
#define LAUNCH_CMD_BYTE 0x20
#define ARM_CMD_BYTE 0x21
#define ABORT_CMD_BYTE 0x2F
#define RESET_CMD_BYTE 0x4F
#define HEARTBEAT_BYTE 0x46
#define OPEN_INJECTION_VALVE 0x2A
#define CLOSE_INJECTION_VALVE 0x2B
#define FLIGHT_PHASE_CHECK_PERIOD (1000)
#define BURN_TO_COST_TIME (5550) //5.55s
#define COAST_TO_POST_TIME (10 * 60 * 1000) // 10 min
#define HEARTBEAT_COUNTDOWN (3 * 60 * 1000) // 3 min
#define LAUNCH_COUNT (3)


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
	ABORT_NO_HEARTBEAT,
    ABORT_COMMUNICATION_ERROR,
    ABORT_OXIDIZER_PRESSURE,
    ABORT_UNSPECIFIED_REASON
} FlightPhase;

/*
 * Switches to the new flight phase specified by newPhase
 */
void newFlightPhase(FlightPhase newPhase);

/*
 * Returns the current flight phase
 */
FlightPhase getCurrentFlightPhase();

/*
 * Resets flight phase to PRELAUNCH
 */
void resetFlightPhase();

/*
 * Check if current flight phase is in one of the abort phases
 */
int isAbortPhase();

/*
 * Thread task to handle commands received from Ground Systems accumulated in flightPhaseQueue
 */
void flightPhaseTask(void const* flightPhaseQueue);

/*
 * Thread task to receive commands from Ground Systems via ESP UART
 */
void gsListenerTask(void const* arg);
