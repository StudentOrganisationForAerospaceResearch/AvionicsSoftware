/**
 ******************************************************************************
 * File Name          : MEVManager.hpp
 * Description        : Servo State and Control Actions
 ******************************************************************************
*/

#ifndef MEV_MANAGER_HPP
#define MEV_MANAGER_HPP

#include "SystemDefines.hpp"
#include "DMBProtocolTask.hpp"
#include "PBBRxProtocolTask.hpp"

class MEVManager {
public:
    enum MEVState {
        INDETERMINATE,
        OPEN,
        CLOSE
    };

    static void OpenMEV();
    static void CloseMEV();
    static void HandleMEVTelemetry(Proto::TelemetryMessage& msg);

private:
    static MEVState shouldMevBeOpen;
};

#endif // MEV_MANAGER_HPP
