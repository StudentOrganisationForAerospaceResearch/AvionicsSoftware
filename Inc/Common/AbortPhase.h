#pragma once

extern uint8_t launchCmdReceived;
extern uint8_t abortCmdReceived;
extern uint8_t resetAvionicsCmdReceived;


void abortPhaseTask(void const* arg);
