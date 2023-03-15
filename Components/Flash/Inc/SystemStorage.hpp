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

#define INITIAL_SENSOR_FLASH_OFFSET 12288

/* Macros/Enums ------------------------------------------------------------*/
enum FLASH_COMMANDS  {
    WRITE_STATE_TO_FLASH = 0,
    DUMP_FLASH_DATA,
    ERASE_ALL_FLASH
};


/**
 * @brief State information to be written to flash
 */
struct StateInformation 
{   
  RocketState State;
  uint32_t SequenceNumber;      
  uint32_t data_offset;
  //uint32_t CRC; 
};

/**
 * @brief System information object
 */
class SystemStorage 
{
public:
    SystemStorage();

    void HandleCommand(Command& cm);

    void WriteStateToFlash();
    bool ReadStateFromFlash();
    void WriteDataToFlash(uint8_t* data, uint16_t size);
    bool ReadDataFromFlash();

protected:

    // Variables
    StateInformation rs_currentInformation;
};

#endif    // SOAR_SYSTEMSTORAGE_HPP_