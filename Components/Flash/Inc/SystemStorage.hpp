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
 * @brief Sensor information to be written to flash
 */
struct SensorInformation 
{   
    uint32_t     offset;
    uint32_t     accelX_;
    uint32_t     accelY_;
    uint32_t     accelZ_;
    uint32_t     gyroX_;
    uint32_t     gyroY_;
    uint32_t     gyroZ_;
    uint32_t     magnetoX_;
    uint32_t     magnetoY_;
    uint32_t     magnetoZ_;   
    uint32_t     pressure_;
    uint32_t     temperature_;
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
    SensorInformation si_currentInformation;

};

#endif    // SOAR_SYSTEMSTORAGE_HPP_