/**
 ******************************************************************************
 * File Name          : FlashTask.cpp
 * Description        : Flash interface task. Used for logging, writing to system
 *                      state, and flash maintenance operations for the system.
 ******************************************************************************
*/
#include "FlashTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Utils.hpp"
#include "Timer.hpp"
#include "RocketSM.hpp"
#include "SPIFlash.hpp"
#include "Data.h"
#include <cstring>

/**
 * @brief Constructor for FlashTask
 */
FlashTask::FlashTask() : Task(FLASH_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the FlashTask
 */
void FlashTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flash task twice");
    
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FlashTask::RunTask,
            (const char*)"FlashTask",
            (uint16_t)FLASH_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)FLASH_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "FlashTask::InitTask() - xTaskCreate() failed");

    SOAR_PRINT("Flash Task initialized");
}

/**
 * @brief Instance Run loop for the Flash Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlashTask::Run(void * pvParams)
{
    // Wait until the flash has been initialized by flight task
    while (!SPIFlash::Inst().GetInitialized())
        osDelay(1);

    // Initialize the offsets storage
    offsetsStorage_ = new SimpleDualSectorStorage<Offsets>(&SPIFlash::Inst(), SPI_FLASH_OFFSETS_SDSS_START_ADDR);
    offsetsStorage_->Read(currentOffsets_);

    while (1) {
        //Process any commands in the queue
        Command cm;
        bool res = qEvtQueue->Receive(cm, MAX_FLASH_TASK_WAIT_TIME_MS);
        if(res)
            HandleCommand(cm);

        //Run maintenance on dual sector storages
        SystemStorage::Inst().Maintain();
        offsetsStorage_->Maintain();
    }
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void FlashTask ::HandleCommand(Command& cm)
{
    // Can't have this if we try to erase then reset?
    //SOAR_ASSERT(w25qxx.UniqID[0] != 0, "Flash command received before flash was initialized");

    switch (cm.GetCommand()) {
    case TASK_SPECIFIC_COMMAND: {
        if (cm.GetTaskCommand() == WRITE_STATE_TO_FLASH)
        {
            // Read the current state from the system storage, and change the rocket state to the new state
            SystemState sysState;
            SystemStorage::Inst().Read(sysState);
            sysState.rocketState = (RocketState)(cm.GetDataPointer()[0]);
            SystemStorage::Inst().Write(sysState);

            SOAR_PRINT("System state written to flash\n");
        }
        else if (cm.GetTaskCommand() == DUMP_FLASH_DATA)
        {
            ReadLogDataFromFlash();
        }
        else if (cm.GetTaskCommand() == ERASE_ALL_FLASH)
        {
            // Erase the system storage to make sure it isn't being used anymore
            SystemStorage::Inst().Erase();

            // Erase the chip
            W25qxx_EraseChip();
            currentOffsets_.writeDataOffset = 0;
        }
        else 
        {
            SOAR_PRINT("FlashTask Received Unsupported Task Command: %d\n", cm.GetTaskCommand());
        }
        break;
    }
    case DATA_COMMAND: {
        // If the command is not a WRITE_DATA_TO_FLASH command do nothing
        if (cm.GetTaskCommand() != WRITE_DATA_TO_FLASH) {
            SOAR_PRINT("FlashTask Received Unsupported Data Command: %d\n", cm.GetTaskCommand());
            break;
        }

        WriteLogDataToFlash(cm.GetDataPointer(), cm.GetDataSize());
        break;
    }
    default:
        SOAR_PRINT("FlashTask Received Unsupported Command: %d\n", cm.GetCommand());
        break;
    }

    cm.Reset();
}

/**
 * @brief writes data to flash with the size of the data written as the header, increases offset by size + 1 to account for size, currently only handles size < 255
 */
void FlashTask::WriteLogDataToFlash(uint8_t* data, uint16_t size)
{
    uint8_t buff[size + 1];

    buff[0] = (uint8_t)(size & 0xff);
    memcpy(buff + 1, data, size);

    SPIFlash::Inst().Write(SPI_FLASH_LOGGING_STORAGE_START_ADDR + currentOffsets_.writeDataOffset, buff, size + 1);
    currentOffsets_.writeDataOffset += size + 1;

    //TODO: Consider adding a readback to check if it was successful

    //If the number of writes since the last offset update has exceeded the threshold, update the offsets in storage
    if(++writesSinceLastOffsetUpdate_ >= FLASH_OFFSET_WRITES_UPDATE_THRESHOLD)
        offsetsStorage_->Write(currentOffsets_);
}

/**
 * @brief reads all data and prints it through UART up until offset read from struct
 *        currently unimplemented
 */
bool FlashTask::ReadLogDataFromFlash()
{
    //unused
    bool res = true;

    uint8_t length;

    for (unsigned int i = 0; i < currentOffsets_.writeDataOffset + SPI_FLASH_LOGGING_STORAGE_START_ADDR; i++) {
        W25qxx_ReadByte(&length, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i);

        if (length == sizeof(AccelGyroMagnetismData)) {
            uint8_t dataRead[sizeof(AccelGyroMagnetismData)];
            W25qxx_ReadBytes(dataRead, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1, sizeof(AccelGyroMagnetismData));
            AccelGyroMagnetismData* IMURead = (AccelGyroMagnetismData*)dataRead;
            SOAR_PRINT("%03d %08d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\n",
                length, IMURead->time, IMURead->accelX_, IMURead->accelY_, IMURead->accelZ_,
                IMURead->gyroX_, IMURead->gyroY_, IMURead->gyroZ_, IMURead->magnetoX_,
                IMURead->magnetoY_, IMURead->magnetoZ_);
        }
        else if (length == sizeof(BarometerData)) {
            uint8_t dataRead[sizeof(BarometerData)];
            W25qxx_ReadBytes(dataRead, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1, sizeof(BarometerData));
            BarometerData* baroRead = (BarometerData*)dataRead;
            SOAR_PRINT("%3d %08d   %04d   %04d\n",
                length, baroRead->time, baroRead->pressure_, baroRead->temperature_);
        }
        else {
            SOAR_PRINT("Unknown length, readback brokedown: %d\n", length);
        }
        i = i + length;
    }
    return res;
}
