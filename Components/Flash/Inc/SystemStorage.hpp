/**
 ******************************************************************************
 * File Name          : FlashTask.hpp
 * Description        : Flash interface task. Used for data logging and state recovery
 ******************************************************************************
*/
#ifndef SOAR_SYSTEMSTORAGE_HPP_
#define SOAR_SYSTEMSTORAGE_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "w25qxx.hpp"

/**
 * @brief State information to be written to flash
 */
struct StateInformation 
{   
  RocketState State;
  uint32_t SequenceNumber;       
}; 

/**
 * @brief System information object
 */
class SystemStorage 
{
public:
    SystemStorage();

    void HandleCommand(Command& cm);

protected:
    bool WriteStateToFlash();
    bool ReadStateFromFlash();

    // Variables
    StateInformation rs_currentInformation;

};

#endif    // SOAR_SYSTEMSTORAGE_HPP_