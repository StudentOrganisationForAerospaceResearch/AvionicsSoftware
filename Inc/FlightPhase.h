/**
  ******************************************************************************
  * File Name          : FlightPhase.h
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Macros --------------------------------------------------------------------*/

/* Externs -------------------------------------------------------------------*/
extern osMutexId flightPhaseMutex;

/* Structs -------------------------------------------------------------------*/
typedef enum
{
    PRELAUNCH,
    BURN,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    ABORT
} FlightPhase;

/* Prototypes ----------------------------------------------------------------*/
void newFlightPhase(FlightPhase newPhase);
FlightPhase getCurrentFlightPhase();
void resetFlightPhase();

