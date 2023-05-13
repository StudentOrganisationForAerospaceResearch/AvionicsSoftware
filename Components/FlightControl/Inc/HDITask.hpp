/**
 ******************************************************************************
 * File Name          : HDITask.hpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_HDI_HPP_
#define SOAR_HDI_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
//#include "FlightTask.hpp"


class HDITask : public Task
{
public:
    static HDITask& Inst() {
        static HDITask inst;
        return inst;
    }

    void InitTask();

    RocketState currentHDIState();

protected:
    static void RunTask(void* pvParams) { HDITask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code


private:
    // Private Functions
    HDITask();        // Private constructor
    HDITask(const HDITask&);                        // Prevent copy-construction
    HDITask& operator=(const HDITask&);            // Prevent assignment

};

#endif    // SOAR_FLIGHTTASK_HPP_
