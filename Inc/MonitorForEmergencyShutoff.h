/**
  ******************************************************************************
  * File Name          : MonitorForEmergencyShutoff.h
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/

/* Externs -------------------------------------------------------------------*/
extern uint8_t abortCmdReceived;
const extern int32_t HEARTBEAT_TIMEOUT;
extern int32_t heartbeatTimer;

/* Structs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/
void monitorForEmergencyShutoffTask(void const* arg);
void monitorForEmergencyShutoffTask(void const* arg);
int prelaunchChecks();
int postArmChecks();
