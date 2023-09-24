#include "MEVManager.hpp"
#include "PBBRxProtocolTask.hpp"
#include "CommandMessage.hpp"
#include "main.h"

MEVManager::MEVState MEVManager::shouldMevBeOpen = INDETERMINATE;

void MEVManager::OpenMEV() {
    shouldMevBeOpen = OPEN;
    HAL_GPIO_WritePin(DRAIN_CONTROL_GPIO_Port, DRAIN_CONTROL_Pin, GPIO_PIN_RESET);
}

void MEVManager::CloseMEV() {
    shouldMevBeOpen = CLOSE;
    HAL_GPIO_WritePin(DRAIN_CONTROL_GPIO_Port, DRAIN_CONTROL_Pin, GPIO_PIN_SET);
}

bool MEVManager::IsMevOpen() {
	return (shouldMevBeOpen == OPEN);
}

void MEVManager::HandleMEVTelemetry(Proto::TelemetryMessage& msg) {
    // The current version of the MEV manager does not run this
	return;
}
