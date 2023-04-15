/**
 ******************************************************************************
 * File Name          : Command.cpp
 * Description        : Command contains the core component base class for all tasks.
 *
 * The order of usage for command memory requires that whenever a command is pulled out from a queue
 * you MUST call Reset() on the command. This will free any memory that was allocated for the command if
 * it is necessary, the logic is internal.
 ******************************************************************************
*/
#include "Command.hpp"
#include "SystemDefines.hpp"

#include <cstring>     // Support for memcpy

/* Static Variable Init ------------------------------------------------------------------*/
std::atomic<uint16_t> Command::statAllocationCounter; // Static variable init



/* Function Implementation ------------------------------------------------------------------*/

/**
 * @brief Default constructor for Command
*/
Command::Command(void)
{
    command = COMMAND_NONE;
    taskCommand = 0;
    data = nullptr;
    dataSize = 0;
    bShouldFreeData = false;
}

/**
 * @brief Constructor with GLOBAL_COMMANDS param
 * @param command GLOBAL_COMMANDS param to initiate command with
*/
Command::Command(GLOBAL_COMMANDS command)
{
    this->command = command;
    taskCommand = 0;
    data = nullptr;
    dataSize = 0;
    bShouldFreeData = false;
}

/**
 * @brief Command with taskCommand field
 * @param taskCommand Task specific command
*/
Command::Command(uint16_t taskCommand)
{
    this->command = TASK_SPECIFIC_COMMAND;
    this->taskCommand = taskCommand;
    data = nullptr;
    dataSize = 0;
    bShouldFreeData = false;
}

/**
 * @brief Constructor with GLOBAL_COMMANDS and taskCommand params
 * @param command GLOBAL_COMMANDS param to initiate command with
 * @param taskCommand taskCommand param to initiate command with
*/
Command::Command(GLOBAL_COMMANDS command, uint16_t taskCommand)
{
    this->command = command;
    this->taskCommand = taskCommand;
    data = nullptr;
    dataSize = 0;
    bShouldFreeData = false;
}

// We cannot use a Destructor, it would get destroyed at lifetime end
//Command::~Command()
//{
//    if(bShouldFreeData && data != nullptr) {
//        delete data; - Not this, pvPortFree
//    }
//}

/**
 * @brief Dynamically allocates memory for the command with the given data size
 * @param dataSize Size of array to allocate
 * @return Pointer to data on success, nullptr on failure (mem already allocated)
*/
uint8_t* Command::AllocateData(uint16_t dataSize)
{
    // If we don't have anything allocated, allocate and return success
    if (this->data == nullptr && !bShouldFreeData) {
        this->data = soar_malloc(dataSize);
        this->bShouldFreeData = true;
        this->dataSize = dataSize;
        statAllocationCounter += 1;

        //TODO: May want to print out whenever we have an imbalance in statAllocationCounter by more than ~5 or so.
        SOAR_ASSERT(statAllocationCounter < MAX_NUMBER_OF_COMMAND_ALLOCATIONS);
        return this->data;
    }
    return nullptr;
}

/**
 * @brief Sets the internal command data pointer to the given data pointer and given size
 *        Only use this for static buffers in which the data does NOT need to be freed.
 *        This may be subject to require synchronization across different threads as the memory
 *        is not indepedent to the Command object.
 * @param existingPtr byte pointer to the data
 * @param size Size of the given data address
 * @return TRUE on success, FALSE on failure (mem already allocated)
*/
bool Command::SetCommandToStaticExternalBuffer(uint8_t* existingPtr, uint16_t size)
{
    // If we don't have anything allocated, set it and return success
    if(this->data == nullptr) {
        this->data = existingPtr;
        this->bShouldFreeData = false;
        this->dataSize = size;
        return true;
    }
    return false;
}

/**
 * @brief Copies data from the source array into memory owned by Command and sets the internal data pointer to the new array
 */
bool Command::CopyDataToCommand(uint8_t* dataSrc, uint16_t size)
{
    // If we successfully allocate, copy the data and return success
    if(this->AllocateData(size)
        && this->data != nullptr) {
        memcpy(this->data, dataSrc, size);
        return true;
    }

    return false;
}

/**
 * @brief Resets command, equivalent of a destructor that must be called, counts allocations and deallocations, asserts an error if the allocation count is too high
*/
void Command::Reset()
{
    if(bShouldFreeData && data != nullptr) {
        soar_free(data);
        statAllocationCounter -= 1;
		data = nullptr;
        bShouldFreeData = false;
    }
}

/**
 * @brief Getter for Data size
 * @return data size if data is allocated, otherwise returns 0 
*/
uint16_t Command::GetDataSize() const
{
    if (data == nullptr)
        return 0;
    return dataSize;
}
