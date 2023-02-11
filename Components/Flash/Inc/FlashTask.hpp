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
    SystemStorage* st_;
    
};

#endif    // SOAR_FLASHTASK_HPP_
