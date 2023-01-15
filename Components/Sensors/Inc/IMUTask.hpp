/**
 ******************************************************************************
 * File Name          : IMUTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_IMU_TASK_HPP_
#define SOAR_SENSOR_IMU_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "Data.h"
#include "SystemDefines.hpp"


/* Macros/Enums ------------------------------------------------------------*/
enum IMU_TASK_COMMANDS {
    IMU_NONE = 0,
    IMU_REQUEST_NEW_SAMPLE,// Get a new IMU sample, task will be blocked for polling time
    IMU_REQUEST_TRANSMIT,    // Send the current IMU data over the Radio
    IMU_REQUEST_DEBUG        // Send the current IMU data over the Debug UART
};

/* Class ------------------------------------------------------------------*/
class IMUTask : public Task
{
public:
    static IMUTask& Inst() {
        static IMUTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { IMUTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams);    // Main run code
    
    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    // Sampling
    void SampleIMU();

    // Setup Functions
    uint8_t SetupIMU();

    // Data
    AccelGyroMagnetismData* data;

private:
    IMUTask();                                        // Private constructor
    IMUTask(const IMUTask&);                    // Prevent copy-construction
    IMUTask& operator=(const IMUTask&);            // Prevent assignment
};

#endif    // SOAR_SENSOR_IMU_TASK_HPP_
