/**
 ******************************************************************************
 * File Name          : BarometerTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_BAROMETER_TASK_HPP_
#define SOAR_SENSOR_BAROMETER_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Data.h"
#include "SystemDefines.hpp"
#include "Task.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum BARO_TASK_COMMANDS {
  BARO_NONE = 0,
  BARO_REQUEST_NEW_SAMPLE,  // Get a new barometer sample, task will be blocked for polling time
  BARO_REQUEST_TRANSMIT,  // Send the current barometer data over the Radio and Log to Flash
  BARO_REQUEST_DEBUG,  // Send the current barometer data over the Debug UART
  BARO_REQUEST_FLASH_LOG,  // Log the current barometer data to flash
};

/* Class ------------------------------------------------------------------*/
class BarometerTask : public Task {
 public:
  static BarometerTask& Inst() {
    static BarometerTask inst;
    return inst;
  }

  void InitTask();

 protected:
  static void RunTask(void* pvParams) {
    BarometerTask::Inst().Run(pvParams);
  }  // Static Task Interface, passes control to the instance Run();

  void Run(void* pvParams);  // Main run code

  void HandleCommand(Command& cm);
  void HandleRequestCommand(uint16_t taskCommand);

  // Telemetry
  void TransmitProtocolBaroData();
  void LogDataToFlash();

  // Sampling
  void SampleBarometer();
  uint16_t ReadCalibrationCoefficients(uint8_t PROM_READ_CMD);

  // Data
  BarometerData* data;

 private:
  BarometerTask();                                 // Private constructor
  BarometerTask(const BarometerTask&);             // Prevent copy-construction
  BarometerTask& operator=(const BarometerTask&);  // Prevent assignment
};

#endif  // SOAR_SENSOR_BAROMETER_TASK_HPP_
