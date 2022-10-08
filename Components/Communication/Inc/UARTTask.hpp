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
enum UART_TASK_COMMANDS {
	UART_TASK_COMMAND_NONE = 0,
	UART_TASK_COMMAND_SEND_DEBUG,
	UART_TASK_COMMAND_MAX
};


/* Class ------------------------------------------------------------------*/
class UARTTask : public Task
{
public:
	static UARTTask& Inst() {
		static UARTTask inst;
		return inst;
	}

	void InitTask();

protected:
	static void RunTask(void* pvParams) { UARTTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	void Run(void* pvParams);	// Main run code

	void ConfigureUART();
	void HandleCommand(Command& cm);

private:
	UARTTask() : Task(UART_TASK_QUEUE_DEPTH_OBJS) {}	// Private constructor
	UARTTask(const UARTTask&);						// Prevent copy-construction
	UARTTask& operator=(const UARTTask&);			// Prevent assignment
};


/* Utility Functions ------------------------------------------------------------------*/
namespace UARTUtils
{
	
}


#endif	// SOAR_COMMS_UARTTASK_HPP_
