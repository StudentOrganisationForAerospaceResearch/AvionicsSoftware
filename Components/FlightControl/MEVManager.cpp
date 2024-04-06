#include "MEVManager.hpp"
#include "PBBRxProtocolTask.hpp"
#include "CommandMessage.hpp"
#include "GPIO.hpp"

MEVManager::MEVState MEVManager::shouldMevBeOpen = INDETERMINATE;

void MEVManager::MEV_OPEN() {
    shouldMevBeOpen = OPEN;
    GPIO::_MAIN_ENGINE_VALVE::Open();
}

void MEVManager::MEV_CLOSE() {
    shouldMevBeOpen = CLOSE;
    GPIO::_MAIN_ENGINE_VALVE::Close();
}
