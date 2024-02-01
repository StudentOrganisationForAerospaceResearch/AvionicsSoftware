	/**
 ******************************************************************************
 * File Name          : PressureTransducerTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_PRESSURE_TRANSDUCER_TASK_HPP_
#define SOAR_SENSOR_PRESSURE_TRANSDUCER_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "Data.h"
#include "SystemDefines.hpp"


/* Macros/Enums ------------------------------------------------------------*/
enum PT_TASK_COMMANDS {
    PT_NONE = 0,
    PT_REQUEST_NEW_SAMPLE,// Get a new pressure transducer sample, task will be blocked for polling time
    PT_REQUEST_TRANSMIT,    // Send the current pressure transducer data over the Radio
    PT_REQUEST_DEBUG,        // Send the current pressure transducer data over the Debug UART
	PT_REQUEST_FLASH_LOG,	// Log current pressure transducer data to flash
	PT_REQUEST_SAMPLE_TRANSMIT_FLASH // Get sample, transmit, and log to flash
};

/* Class ------------------------------------------------------------------*/
class PressureTransducerTask : public Task
{
public:
    static PressureTransducerTask& Inst() {
        static PressureTransducerTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { PressureTransducerTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams);    // Main run code
    
    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    // Sampling
    void SamplePressureTransducer();
    void TransmitProtocolPressureData();
    void LogPressure();

    // Data
    PressureTransducerData* data;
    uint32_t timestampPT;
    uint32_t numLogsSinceTransmit;

    uint32_t lastPTCProtobufTick;

private:
    PressureTransducerTask();                                        // Private constructor
    PressureTransducerTask(const PressureTransducerTask&);                    // Prevent copy-construction
    PressureTransducerTask& operator=(const PressureTransducerTask&);            // Prevent assignment
};

#endif    // SOAR_SENSOR_PRESSURE_TRANSDUCER_TASK_HPP_
