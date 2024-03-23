#include "MEVManager.hpp"
#include "PBBRxProtocolTask.hpp"
#include "CommandMessage.hpp"

MEVManager::MEVState MEVManager::shouldMevBeOpen = INDETERMINATE;

void MEVManager::OpenMEV() {
    shouldMevBeOpen = OPEN;
    PBBRxProtocolTask::SendPbbCommand(Proto::PbbCommand::Command::PBB_OPEN_MEV);
}

void MEVManager::CloseMEV() {
    shouldMevBeOpen = CLOSE;
    PBBRxProtocolTask::SendPbbCommand(Proto::PbbCommand::Command::PBB_CLOSE_MEV);
}

void MEVManager::HandleMEVTelemetry(Proto::TelemetryMessage& msg) {
    if (shouldMevBeOpen == INDETERMINATE || !msg.has_combustionControlStatus()) {
        // Do nothing
        return;
    }

    if (!msg.combustionControlStatus().mev_open() && shouldMevBeOpen == OPEN) {
        OpenMEV();
    } else if (msg.combustionControlStatus().mev_open() && shouldMevBeOpen == CLOSE) {
        CloseMEV();
    }
}