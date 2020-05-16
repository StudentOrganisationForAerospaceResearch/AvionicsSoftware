/**
  ******************************************************************************
  * File Name          : AbortPhase.h
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/
#define ABORT_VALVE_PULSE_MAX_TIME (8000)

/* Externs -------------------------------------------------------------------*/
extern uint8_t launchCmdReceived;
extern uint8_t systemIsArmed;
extern uint8_t pulseVentValveRequested;
extern uint8_t abortCmdReceived;
extern uint8_t resetAvionicsCmdReceived;

const extern int32_t HEARTBEAT_TIMEOUT;
extern int32_t heartbeatTimer;

/* Structs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/
void abortPhaseTask(void const* arg);
