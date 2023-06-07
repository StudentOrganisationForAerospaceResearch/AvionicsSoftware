/**
 ******************************************************************************
 * File Name          : FlashTask.hpp
 * Description        : Flash interface task. Used for data logging and state recovery
 ******************************************************************************
*/
#ifndef SOAR_FLASHTASK_HPP_
#define SOAR_FLASHTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "SystemStorage.hpp"
#include "SPIFlash.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum FLASH_TASK_COMMANDS {
    FLASH_NONE = 0,
    FLASH_WRITE_STATE,// write state to flash
    FLASH_WRITE_SENSOR,    // write new sensor data to flash
    FLASH_DUMP_SENSOR        // dump all sensor data in flash throguh UART
};

class FlashTask : public Task
{
public:
    static FlashTask& Inst() {
        static FlashTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { FlashTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code


private:
    // Private Functions
    FlashTask();        // Private constructor
    FlashTask(const FlashTask&);                        // Prevent copy-construction
    FlashTask& operator=(const FlashTask&);            // Prevent assignment

    // Private Variables
    SPIFlash spiFlash_;

    SystemStorage* st_;
    
};

#endif    // SOAR_FLASHTASK_HPP_
