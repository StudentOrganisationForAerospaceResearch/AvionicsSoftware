	/**
 ******************************************************************************
 * File Name          : BatteryTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_BATTERY_TASK_HPP_
#define SOAR_SENSOR_BATTERY_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "Data.h"
#include "SystemDefines.hpp"
#include "TelemetryMessage.hpp"


/* Macros/Enums ------------------------------------------------------------*/
enum BATTERY_TASK_COMMANDS {
    BATTERY_NONE = 0,
    BATTERY_REQUEST_NEW_SAMPLE,// Get a new battery voltage sample, task will be blocked for polling time
    BATTERY_REQUEST_TRANSMIT,    // Send the current battery voltage data over the Radio
    BATTERY_REQUEST_DEBUG,        // Send the current battery voltage data over the Debug UART
};

enum power_source {
        INVALID = 0,
        GROUND = 1,
        ROCKET = 2,
};

/* Class ------------------------------------------------------------------*/
class BatteryTask : public Task
{
public:
    static BatteryTask& Inst() {
        static BatteryTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { BatteryTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams);    // Main run code
    
    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    // Sampling
    void SampleBatteryVoltage();
    void TransmitProtocolBatteryData();
    enum Proto::Battery::power_source GetPowerState();

    // Data
    BatteryData* data;
    uint32_t timestampPT;

private:
    BatteryTask();                                        // Private constructor
    BatteryTask(const BatteryTask&);                    // Prevent copy-construction
    BatteryTask& operator=(const BatteryTask&);            // Prevent assignment
};

#endif    // SOAR_SENSOR_BATTERY_TASK_HPP_
