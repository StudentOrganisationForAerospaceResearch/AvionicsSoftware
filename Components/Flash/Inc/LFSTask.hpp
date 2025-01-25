/**
 ********************************************************************************
 * @file    LFSTask.hpp
 * @author  spiro
 * @date    Jan 25, 2025
 * @brief
 ********************************************************************************
 */

#ifndef FLASH_INC_LFSTASK_HPP_
#define FLASH_INC_LFSTASK_HPP_

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
void SOAR_PRINT(const char* format, ...); // Added prototype


#include "Task.hpp"
#include "SystemDefines.hpp"
#include "SystemStorage.hpp"
#include "SPIFlash.hpp"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint16_t MAX_FLASH_TASK_WAIT_TIME_MS = 5000; // The max time to wait for a command before maintenance is checked
constexpr uint8_t FLASH_OFFSET_WRITES_UPDATE_THRESHOLD = 50; // The number of writes to flash before offsets are updated in flash


enum FLASH_COMMANDS {
    WRITE_STATE_TO_FLASH = 0,
    WRITE_DATA_TO_FLASH = 0x31,
    DUMP_FLASH_DATA = 0x50,
    ERASE_ALL_FLASH = 0x60,
};


class LFSTask : public Task
{
public:
    static LFSTask& Inst() {
        static LFSTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { LFSTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

    // Log Data Functions
    void WriteLogDataToFlash(uint8_t* data, uint16_t size);
    bool ReadLogDataFromFlash();

private:


    // Private Functions
    LFSTask();        // Private constructor
    LFSTask(const LFSTask&);                        // Prevent copy-construction
    LFSTask& operator=(const LFSTask&);            // Prevent assignment

    // Offsets
    struct Offsets
    {
        uint32_t writeDataOffset;
    };

    Offsets currentOffsets_;
    SimpleDualSectorStorage<Offsets>* offsetsStorage_;

    uint8_t writesSinceLastOffsetUpdate_;
};

#endif
