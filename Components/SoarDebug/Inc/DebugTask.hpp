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

/* Enums ------------------------------------------------------------------*/
enum DEBUG_TASK_COMMANDS {
	DEBUG_TASK_COMMAND_NONE = 0,
	EVENT_DEBUG_RX_COMPLETE
};

/* Macros ------------------------------------------------------------------*/
constexpr uint16_t DEBUG_RX_BUFFER_SZ_BYTES = 16;

/* Class ------------------------------------------------------------------*/
class DebugTask : public Task
{
public:
	static DebugTask& Inst() {
		static DebugTask inst;
		return inst;
	}

	void InitTask();

	//Functions exposed to HAL callbacks
	void InterruptRxData();

protected:
	static void RunTask(void* pvParams) { DebugTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	void Run(void* pvParams);	// Main run code

	void ConfigureUART();
	void HandleDebugMessage(const char* msg);
	//void HandleCommand(Command& cm);

	bool ReceiveData();

	// Helper functions
	static int32_t ExtractIntParameter(const char* msg, uint16_t identifierLen);
	
	// Member variables
	uint8_t debugBuffer[DEBUG_RX_BUFFER_SZ_BYTES+1];
	uint8_t debugMsgIdx;
	bool isDebugMsgReady;

	uint8_t debugRxChar; // Character received from UART Interrupt

private:
	DebugTask(); // Private constructor
	DebugTask(const DebugTask&);					// Prevent copy-construction
	DebugTask& operator=(const DebugTask&);			// Prevent assignment
};

#endif	// SOAR_SYSTEM_DEBUG_TASK_HPP_
