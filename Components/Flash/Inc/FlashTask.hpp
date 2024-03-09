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

#define SHIFTED_FLASH_TASK_LOG_TYPE(LTYPE) ((LTYPE << 5)&0b11100000)
#define PAGECRCLEN 2

enum FLASH_COMMANDS {
    WRITE_STATE_TO_FLASH = 0,
    WRITE_DATA_TO_FLASH = 0x01, // The top 3 bits of this contain the ID of the log type to be logged
    DUMP_FLASH_DATA = 0x02,
    ERASE_ALL_FLASH = 0x03,
	GET_FLASH_OFFSET = 0x04,
	FLASH_DUMP_AT = 0x05,
	FLASH_RESET_AND_ERASE = 0x06,
	GET_LOGS_PAST_SECOND = 0x07,
	GET_PAGE_OFFSET = 0x08,
	FLASH_READ_FIRST_LOGS = 0x09,
	TOG_BUFLOGS = 0x0A
};

enum FLASH_LOG_TYPE { // at most 8 types
	LTYPE_INVAL,
	LTYPE_BAROMETER,
	LTYPE_ACCELGYROMAG,
	LTYPE_PTC,
	LTYPE_GPS,
	LTYPE_PBB_PRES,
	LTYPE_MEV_STATE,
	LTYPE_OTHER
};

class FlashTask : public Task
{
public:
    static FlashTask& Inst() {
        static FlashTask inst;
        return inst;
    }

    void InitTask();


    // Will put cmd in regular queue, unless it is full, where it will put cmd in the
    // separate priority queue
    void SendPriorityCommand(Command& cmd);

protected:
    static void RunTask(void* pvParams) { FlashTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

    // Log Data Functions
    void WriteLogDataToFlash(uint8_t* data, uint16_t size);
    void WriteLogDataToFlashPageAligned(uint8_t* data, uint16_t size,uint32_t pageAddr);
    bool ReadLogDataFromFlash();
    void AddLog(FLASH_LOG_TYPE type, const uint8_t* datain, uint32_t size);
    bool DumpFirstNLogs(uint32_t numOfLogs, bool verbose);

    bool writebuftimemsg = false;

    Queue* qPriorityQueue;


    static void benchmarkcallback(TimerHandle_t x) {


        Command cmd((uint16_t)GET_LOGS_PAST_SECOND);
        FlashTask::Inst().SendPriorityCommand(cmd);

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
    PBBPressureFlashLogData lastPBBPres;


    uint8_t currentPageStorageByte; // from 0 to pagesize in increments of 8
    uint8_t currentPageStoragePage; // from 0 to 15
    uint8_t currentPageStorageSector; // either 0 or 1


    bool flashDumpVerbose;




};

#endif    // SOAR_FLASHTASK_HPP_
