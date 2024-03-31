#include "MEVManager.hpp"
#include "PBBRxProtocolTask.hpp"
#include "CommandMessage.hpp"
#include "GPIO.hpp"

MEVManager::MEVState MEVManager::shouldMevBeOpen = INDETERMINATE;

void MEVManager::OpenMEV() {
    shouldMevBeOpen = OPEN;
    GPIO::MEV_EN::On();
}

void MEVManager::CloseMEV() {
    shouldMevBeOpen = CLOSE;
    GPIO::MEV_EN::Off();
}

void MEVManager::HandleMEVTelemetry(Proto::TelemetryMessage& msg) {
}
