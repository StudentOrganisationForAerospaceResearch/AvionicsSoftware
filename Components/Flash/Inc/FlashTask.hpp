/**
 ******************************************************************************
 * File Name          : FlashTask.hpp
 * Description        : Flash interface task. Used for data logging and state recovery
 ******************************************************************************
*/
#ifndef SOAR_FLASHTASK_HPP_
#define SOAR_FLASHTASK_HPP_
#include "SPIFlash.hpp"
#include "SystemDefines.hpp"
#include "SystemStorage.hpp"
#include "Task.hpp"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint16_t MAX_FLASH_TASK_WAIT_TIME_MS =
    5000;  // The max time to wait for a command before maintenance is checked
constexpr uint8_t FLASH_OFFSET_WRITES_UPDATE_THRESHOLD =
    50;  // The number of writes to flash before offsets are updated in flash

enum FLASH_COMMANDS {
    WRITE_STATE_TO_FLASH = 0,
    WRITE_DATA_TO_FLASH = 0x31,
    DUMP_FLASH_DATA = 0x50,
    ERASE_ALL_FLASH = 0x60,
};

class FlashTask : public Task {
   public:
    static FlashTask& Inst() {
        static FlashTask inst;
        return inst;
    }

    void InitTask();

   protected:
    static void RunTask(void* pvParams) {
        FlashTask::Inst().Run(pvParams);
    }  // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams);  // Main run code

    void HandleCommand(Command& cm);

    // Log Data Functions
    void WriteLogDataToFlash(uint8_t* data, uint16_t size);
    bool ReadLogDataFromFlash();

   private:
    // Private Functions
    FlashTask();                             // Private constructor
    FlashTask(const FlashTask&);             // Prevent copy-construction
    FlashTask& operator=(const FlashTask&);  // Prevent assignment

    // Offsets
    struct Offsets {
        uint32_t writeDataOffset;
    };

    Offsets currentOffsets_;
    SimpleDualSectorStorage<Offsets>* offsetsStorage_;

    uint8_t writesSinceLastOffsetUpdate_;
};

#endif  // SOAR_FLASHTASK_HPP_
