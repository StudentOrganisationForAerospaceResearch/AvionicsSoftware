/**
 ******************************************************************************
 * File Name          : FlightTask.hpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_FLIGHTTASK_HPP_
#define SOAR_FLIGHTTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"

class FlightTask : public Task
{
public:
	static FlightTask& Inst() {
		static FlightTask inst;
		return inst;
	}

	void InitTask();

protected:
	static void RunTask(void* pvParams) { FlightTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	void Run(void * pvParams); // Main run code


private:
	FlightTask() : Task(FLIGHT_TASK_QUEUE_DEPTH_OBJS) {}		// Private constructor
	FlightTask(const FlightTask&);						// Prevent copy-construction
	FlightTask& operator=(const FlightTask&);			// Prevent assignment
};

#endif	// SOAR_FLIGHTTASK_HPP_
