#include "Command.hpp"

#include "SystemDefines.hpp"
/**
 ******************************************************************************
 * File Name          : Task.cpp
 * Description        : Task contains the core component base class for all tasks.
 ******************************************************************************
*/

/* Static Variable Init ------------------------------------------------------------------*/
uint16_t Command::statAllocationCounter = 0; // Static variable init



/* Function Implementation ------------------------------------------------------------------*/

/**
 * @brief Default constructor
 * @param None
*/
Command::Command(void)
{
    command = COMMAND_NONE;
    taskCommand = 0;
    data = nullptr;
    dataSize = 0;
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
}

/**
 * @brief Command with taskCommand field
 * @param command 
 * @param taskCommand 
*/
Command::Command(uint16_t taskCommand)
{
    this->command = TASK_SPECIFIC_COMMAND;
    this->taskCommand = taskCommand;
    data = nullptr;
    dataSize = 0;
}

//Command::~Command()
//{
//    if(bShouldFreeData && data != nullptr) {
//        delete data; - Not this, pvPortFree
//    }
//}

/**
 * @brief Dynamically allocates memory for the command with the given data size
 * @param dataSize Size of array to allocate
 * @return TRUE on success, FALSE on failure (mem already allocated)
*/
bool Command::AllocateData(uint16_t dataSize)
{
    // If we don't have anything allocated, allocate and return success
    if (this->data == nullptr && !bShouldFreeData) {
        this->data = Malloc(dataSize);
        this->bShouldFreeData = true;
        this->dataSize = dataSize;
        statAllocationCounter += 1;

        //TODO: May want to print out whenever we have an imbalance in statAllocationCounter by more than ~5 or so.
        SOAR_ASSERT(statAllocationCounter < MAX_NUMBER_OF_COMMAND_ALLOCATIONS);
        return true;
    }
    return false;
}

/**
 * @brief Sets the internal command data pointer to the given data pointer and given size
 * @param dataPtr byte pointer to the data
 * @param size Size of the given data address
 * @param bFreeMemory Whether the command packet should try to free
 * @return TRUE on success, FALSE on failure (mem already allocated)
*/
bool Command::SetCommandData(uint8_t* dataPtr, uint16_t size, bool bFreeMemory)
{
    // If we don't have anything allocated, set it and return success
    if(this->data == nullptr && !bShouldFreeData) {
        this->data = dataPtr;
        this->bShouldFreeData = bFreeMemory;
        this->dataSize = size;
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
        Free(data);
        statAllocationCounter -= 1;
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
