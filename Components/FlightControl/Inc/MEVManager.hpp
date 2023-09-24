/**
 ******************************************************************************
 * File Name          : MEVManager.hpp
 * Description        : Servo State and Control Actions
 ******************************************************************************
*/

#ifndef MEV_MANAGER_HPP
#define MEV_MANAGER_HPP

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
    static bool IsMevOpen();

    static inline void EnableMev() { isMevEnabled = true; }
    static inline void DisableMev() { isMevEnabled = false; }
    static inline bool IsMevEnabled() { return isMevEnabled; }

private:
    static bool isMevEnabled;
    static MEVState shouldMevBeOpen;
};

#endif // MEV_MANAGER_HPP
