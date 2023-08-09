#include "MEVManager.hpp"
#include "PBBRxProtocolTask.hpp"
#include "CommandMessage.hpp"


void MEVManager::OpenMEV() {
    shouldMevBeOpen = OPEN;
    PBBRxProtocolTask::SendPBBCommand(Proto::PBBCommand::Command::PBB_OPEN_MEV);
}

void MEVManager::CloseMEV() {
    shouldMevBeOpen = CLOSE;
    PBBRxProtocolTask::SendPBBCommand(Proto::PBBCommand::Command::PBB_CLOSE_MEV);
}

void MEVManager::HandleMEVTelemetry(Proto::TelemetryMessage& msg) {
    if (shouldMevBeOpen == INDETERMINATE || !msg.has_mevstate()) {
        // Do nothing
        return;
    }

    Proto::MEVState mevState = msg.mevstate();
    if (!mevState.mev_open() && shouldMevBeOpen == OPEN) {
        OpenMEV();
    } else if (mevState.mev_open() && shouldMevBeOpen == CLOSE) {
        CloseMEV();
    }
}

MEVManager::MEVState MEVManager::shouldMevBeOpen = INDETERMINATE;
