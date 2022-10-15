/**
 ******************************************************************************
 * File Name          : DebugTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SYSTEM_DEBUG_TASK_HPP_
#define SOAR_SYSTEM_DEBUG_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"


/* Macros ------------------------------------------------------------------*/


/* Class ------------------------------------------------------------------*/
class DebugTask : public Task
{
public:
	static DebugTask& Inst() {
		static DebugTask inst;
		return inst;
	}

	void InitTask();

protected:
	static void RunTask(void* pvParams) { DebugTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	void Run(void* pvParams);	// Main run code

	void ConfigureUART();
	void HandleCommand(Command& cm);

private:
	DebugTask() : Task(TASK_DEBUG_STACK_DEPTH_WORDS) {}	// Private constructor
	DebugTask(const DebugTask&);						// Prevent copy-construction
	DebugTask& operator=(const DebugTask&);			// Prevent assignment
};

#endif	// SOAR_SYSTEM_DEBUG_TASK_HPP_
