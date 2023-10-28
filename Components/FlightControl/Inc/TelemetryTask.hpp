/**
 ******************************************************************************
 * File Name          : TelemetryTask.hpp
 * Description        : Telemetry task controls the rate at which sensors or other
 *                      tasks send data to the ground station.
 ******************************************************************************
*/
#ifndef SOAR_TELEMETRYTASK_HPP_
#define SOAR_TELEMETRYTASK_HPP_
#include "SystemDefines.hpp"
#include "Task.hpp"

constexpr uint8_t NUM_SENT_LOGS_PER_FLASH_LOG =
    3 * 5;  // N cycles of telemetry sends for each flash log

class TelemetryTask : public Task {
 public:
  static TelemetryTask& Inst() {
    static TelemetryTask inst;
    return inst;
  }

  void InitTask();

 protected:
  static void RunTask(void* pvParams) {
    TelemetryTask::Inst().Run(pvParams);
  }  // Static Task Interface, passes control to the instance Run();

  void Run(void* pvParams);  // Main run code

  void HandleCommand(Command& cm);
  void RunLogSequence();

  void RequestSample();
  void RequestTransmit();
  void RequestLogToFlash();

  void SendVentDrainStatus();

 private:
  // Private Functions
  TelemetryTask();                                 // Private constructor
  TelemetryTask(const TelemetryTask&);             // Prevent copy-construction
  TelemetryTask& operator=(const TelemetryTask&);  // Prevent assignment

  // Private Variables
  uint32_t loggingDelayMs;

  uint8_t numNonFlashLogs_;
};

#endif  // SOAR_TELEMETRYTASK_HPP_
