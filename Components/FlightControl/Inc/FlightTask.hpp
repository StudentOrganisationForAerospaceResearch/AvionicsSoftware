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

class FlightTask : Task
{
public:
	static FlightTask& Inst() {
		static FlightTask inst;
		return inst;
	}

	void InitTask();

protected:
	static void Run(void * pvParams); // Task loop


private:
	FlightTask() : Task(FLIGHT_TASK_QUEUE_SIZE) {}	// Private constructor
	FlightTask(const FlightTask&);						// Prevent copy-construction
	FlightTask& operator=(const FlightTask&);			// Prevent assignment
};

#endif	// SOAR_FLIGHTTASK_HPP_