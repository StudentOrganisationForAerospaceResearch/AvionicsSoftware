/**
 ******************************************************************************
 * File Name          : BarometerTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_BAROMETER_TASK_HPP_
#define SOAR_SENSOR_BAROMETER_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"


/* Macros ------------------------------------------------------------------*/


/* Class ------------------------------------------------------------------*/
class BarometerTask : public Task
{
public:
	static BarometerTask& Inst() {
		static BarometerTask inst;
		return inst;
	}

	void InitTask();

protected:
	static void RunTask(void* pvParams) { BarometerTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	void Run(void* pvParams);	// Main run code
	
	void HandleCommand(Command& cm);

private:
	BarometerTask() : Task(TASK_DEBUG_STACK_DEPTH_WORDS) {}	// Private constructor
	BarometerTask(const BarometerTask&);					// Prevent copy-construction
	BarometerTask& operator=(const BarometerTask&);			// Prevent assignment
};

#endif	// SOAR_SENSOR_BAROMETER_TASK_HPP_
