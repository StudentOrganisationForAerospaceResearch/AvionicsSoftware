/**
 ******************************************************************************
 * File Name          : UARTTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_COMMS_UARTTASK_HPP_
#define SOAR_COMMS_UARTTASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"



/* Macros ------------------------------------------------------------------*/



/* Class ------------------------------------------------------------------*/
class UARTTask : Task
{
public:
	static UARTTask& Inst() {
		static UARTTask inst;
		return inst;
	}

	void InitTask();

protected:
	static void Run(void* pvParams); // Task loop


private:
	UARTTask() : Task(FLIGHT_TASK_QUEUE_SIZE) {}	// Private constructor
	UARTTask(const UARTTask&);						// Prevent copy-construction
	UARTTask& operator=(const UARTTask&);			// Prevent assignment


};


/* Utility Functions ------------------------------------------------------------------*/
namespace UARTUtils
{
	
}


#endif	// SOAR_COMMS_UARTTASK_HPP_
