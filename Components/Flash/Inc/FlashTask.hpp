/**
 ******************************************************************************
 * File Name          : FlashTask.hpp
 * Description        : Flash interface task. Used for data logging and state recovery
 ******************************************************************************
*/
#ifndef SOAR_FLASHTASK_HPP_
#define SOAR_FLASHTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "SystemStorage.hpp"
#include "SPIFlash.hpp"
#include "Timer.hpp"
#include "Data.h"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint16_t MAX_FLASH_TASK_WAIT_TIME_MS = 5000; // The max time to wait for a command before maintenance is checked
constexpr uint8_t FLASH_OFFSET_WRITES_UPDATE_THRESHOLD = 50; // The number of writes to flash before offsets are updated in flash
constexpr size_t FLASH_HEAP_BUF_SIZE = 256; // The size in bytes of each buffer for holding incoming sensor data

enum FLASH_COMMANDS {
    WRITE_STATE_TO_FLASH = 0,
    WRITE_DATA_TO_FLASH = 0x31,
    DUMP_FLASH_DATA = 0x50,
    ERASE_ALL_FLASH = 0x60,
	GET_FLASH_OFFSET = 0x70,
	FLASH_DUMP_AT = 0x80,
	FLASH_DEBUGWRITE = 0x90,
	GET_LOGS_PAST_SECOND = 0x99,
	GET_PAGE_OFFSET = 0xa1,
	FLASH_READ_FIRST_LOGS = 0xba,
	TOG_BUFLOGS = 0xbb
};


class FlashTask : public Task
{
public:
    static FlashTask& Inst() {
        static FlashTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { FlashTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

    // Log Data Functions
    void WriteLogDataToFlash(uint8_t* data, uint16_t size);
    void WriteLogDataToFlashPageAligned(uint8_t* data, uint16_t size,uint32_t pageAddr);
    bool ReadLogDataFromFlash();
    void AddLog(const uint8_t* datain, uint32_t size);
    bool DebugReadLogs(uint32_t numOfLogs);

    bool writebuftimemsg = true;


    static void benchmarkcallback(TimerHandle_t x) {


        Command cmd((uint16_t)GET_LOGS_PAST_SECOND);
        FlashTask::Inst().GetEventQueue()->Send(cmd);

    	Timer::DefaultCallback(x);
    }

private:


    // Private Functions
    FlashTask();        // Private constructor
    FlashTask(const FlashTask&);                        // Prevent copy-construction
    FlashTask& operator=(const FlashTask&);            // Prevent assignment

    // Offsets
    struct Offsets
    {
        uint32_t writeDataOffset;
    };

    Offsets currentOffsets_;
    SimpleDualSectorStorage<Offsets>* offsetsStorage_;

    uint8_t writesSinceLastOffsetUpdate_;

    uint8_t* logbufA;
    uint8_t* logbufB;
    uint8_t* currbuf;
    uint8_t offsetWithinBuf;

    uint32_t currentLogPage;



    Timer* benchmarktimer;

    uint16_t logsInLastSecond;

    BarometerData lastBaroData;
    AccelGyroMagnetismData lastIMUData;
    PressureTransducerFlashLogData lastPTC;
//    GPSDataFlashLog lastGPSFlashData;

    uint8_t currentPageStorageByte; // from 0 to pagesize in increments of 8
    uint8_t currentPageStoragePage; // from 0 to 15
    uint8_t currentPageStorageSector; // either 0 or 1





};

#endif    // SOAR_FLASHTASK_HPP_
