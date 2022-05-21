#pragma once

extern uint8_t abortCmdReceived;
const extern int32_t HEARTBEAT_TIMEOUT;
extern int32_t heartbeatTimer;

void monitorForEmergencyShutoffTask(void const* arg);
int prelaunchChecks();
int postArmChecks();
