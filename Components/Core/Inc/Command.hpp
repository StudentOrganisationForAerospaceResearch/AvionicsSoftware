/**
 ******************************************************************************
 * File Name          : Command.hpp
 * Description        : Command is a unique object used to communicate information
 *	to and between tasks.
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_COMMAND_H
#define AVIONICS_INCLUDE_SOAR_CORE_COMMAND_H
/* Includes ------------------------------------------------------------------*/
#include <atomic>

#include "cmsis_os.h"

/* Macros --------------------------------------------------------------------*/

/* Enums -----------------------------------------------------------------*/
enum GLOBAL_COMMANDS : uint8_t
{
	COMMAND_NONE = 0,		// No command, packet can probably be ignored
	TASK_SPECIFIC_COMMAND,	// Runs a task specific command when given this object
	DATA_COMMAND,			// Data command, used to send data to a task. Target is stored in taskCommand
    CONTROL_ACTION,			// Control actions, used in Rocket State Machine, direct translation to RCU<->DMB Protocol
	REQUEST_COMMAND			// Request command
};

/* Class -----------------------------------------------------------------*/

/**
 * @brief Command class
 *
 * Each Command object contains one set of commands, a GLOBAL_COMMANDS and a task command that can be task specific.
 *
 * Note, this class must be able to be treated as 'Plain-Old-Data' as it will be handled with raw-copy in RTOS queues
*/
class Command
{
public:
	Command(void);
	Command(GLOBAL_COMMANDS command);
	Command(uint16_t taskCommand);
	Command(GLOBAL_COMMANDS command, uint16_t taskCommand);

	//~Command();	// We can't handle memory like this, since the object would be 'destroyed' after copying to the RTOS queue

	// Functions
	bool AllocateData(uint16_t dataSize);	// Dynamically allocates data for the command
	bool CopyDataToCommand(uint8_t* dataSrc, uint16_t size);	// Copies the data into the command, into newly allocated memory
	bool SetCommandToStaticExternalBuffer(uint8_t* existingPtr, uint16_t size);	// Set data pointer to a pre-allocated buffer, if bFreeMemory is set to true, responsibility for freeing memory will fall on Command

	void Reset();	// Reset the command, equivalent of a destructor that must be called, counts allocations and deallocations, asserts an error if the allocation count is too high

	// Getters
	uint16_t GetDataSize() const;
	uint8_t* GetDataPointer() const { return data; }
	GLOBAL_COMMANDS GetCommand() const { return command; }
	uint16_t GetTaskCommand() const { return taskCommand; }

	// Setters
	void SetTaskCommand(uint16_t taskCommand) { this->taskCommand = taskCommand; }


protected:
	// Data -- note each insertion and removal from a queue will do a full copy of this object, so this data should be as small as possible
	GLOBAL_COMMANDS command;	// General GLOBAL command, each task must be able to handle these types of commands
	uint16_t taskCommand;		// Task specific command, the task this command event is sent to needs to handle this

	uint8_t* data;				// Pointer to optional data
	uint16_t dataSize;			// Size of optional data

private:
	bool bShouldFreeData;		// Should the Command handle freeing the data pointer (necessary to enable Command object to handle static memory ptrs)

	static std::atomic<uint16_t> statAllocationCounter;	// Static allocation counter shared by all command objects

	Command(const Command&);	// Prevent copy-construction
};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_COMMAND_H */