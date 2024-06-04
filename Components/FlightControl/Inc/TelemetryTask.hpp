/**
 ******************************************************************************
 * File Name          : TelemetryTask.hpp
 * Description        : Telemetry task controls the rate at which sensors or other
 *                      tasks send data to the ground station.
 ******************************************************************************
*/
#ifndef SOAR_TELEMETRYTASK_HPP_
#define SOAR_TELEMETRYTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"

constexpr uint16_t TELEMETRY_HEARTBEAT_TIMER_PERIOD_MS = 2000; // 2s between heartbeat telemetry
constexpr uint16_t PERIOD_BETWEEN_FLASH_LOGS_MS = 10000; // 10s between logs to flash

class TelemetryTask : public Task
{
public:
    static TelemetryTask& Inst() {
        static TelemetryTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { TelemetryTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams); // Main run code

    void HandleCommand(Command& cm);
    void RunLogSequence();

    void RequestSample();
    void RequestTransmit();
    void RequestLogToFlash();

    void SendVentDrainStatus();


private:
    // Private Functions
    TelemetryTask();        // Private constructor
    TelemetryTask(const TelemetryTask&);                        // Prevent copy-construction
    TelemetryTask& operator=(const TelemetryTask&);            // Prevent assignment

    // Private Variables
    uint32_t loggingDelayMs;

    uint16_t numNonFlashLogs_;
    uint16_t numNonControlLogs_;
};

#endif    // SOAR_TELEMETRYTASK_HPP_
