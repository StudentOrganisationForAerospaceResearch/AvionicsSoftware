/*
 * FlashLogHandler.hpp
 *
 *  Created on: May 25, 2024
 *      Author: goada
 */

#ifndef FLASH_INC_FLASHLOGHANDLER_HPP_
#define FLASH_INC_FLASHLOGHANDLER_HPP_

#include <stdint.h>
#include "BarometerTask.hpp"
#include "IMUTask.hpp"
#include "GPSTask.hpp"
#include "PressureTransducerTask.hpp"
#include "Timer.hpp"

constexpr uint8_t FLASH_OFFSET_WRITES_UPDATE_THRESHOLD = 50; // The number of writes to flash before offsets are updated in flash
constexpr size_t FLASH_HEAP_BUF_SIZE = 256; // The size in bytes of each buffer for holding incoming sensor data

#define SHIFTED_FLASH_TASK_LOG_TYPE(LTYPE) ((LTYPE << 5)&0b11100000)
#define PAGECRCLEN 2

#define MIN(a,b) ((a)>(b) ? (b):(a))
#define LTYPE_USES_DELTA_ENCODING(type) ((type) == LTYPE_BAROMETER or (type) == LTYPE_ACCELGYROMAG or (type) == LTYPE_PTC or (type) == LTYPE_PBB_PRES)


enum FLASH_LOG_TYPE { // at most 8 types
    LTYPE_INVAL = 0,
    LTYPE_BAROMETER,
    LTYPE_ACCELGYROMAG,
    LTYPE_PTC,
    LTYPE_GPS,
    LTYPE_PBB_PRES,
    LTYPE_MEV_STATE,
    LTYPE_OTHER
};


class FlashLogHandler {

public:
    FlashLogHandler(uint32_t pageOffsetBackupSectorStart, uint32_t loggingSectorStart, uint32_t logSpaceInSectors, uint32_t sectorSize);
    int AddFlashLog(FLASH_LOG_TYPE type, const uint8_t *datain, uint32_t size);
    void ResetLogging();
    uint32_t GetCurrentPage();
    uint16_t GetLogBenchmark();
    void PartialResetLogging(uint16_t sectorsToErase);
    bool ToggleMessageOnWrite();
    bool RecoverLastWrittenPage(bool clear);
    bool DumpFirstNLogs(uint32_t N, bool verbose);


private:

    FlashLogHandler(const FlashLogHandler&);                        // Prevent copy-construction
    FlashLogHandler& operator=(const FlashLogHandler&);            // Prevent assignment


    uint8_t* logbufA;
    uint8_t* logbufB;
    uint8_t* currbuf;
    uint8_t offsetWithinBuf;


    uint32_t currentLogPage;




    bool writebuftimemsg = false;

    uint16_t logsInLastSecond;

    BarometerData lastBaroData;
    AccelGyroMagnetismData lastIMUData;
    PressureTransducerFlashLogData lastPTC;
    PBBPressureFlashLogData lastPBBPres;


    uint8_t currentPageStorageByte; // from 0 to pagesize in increments of 8
    uint8_t currentPageStoragePage; // from 0 to 15
    uint8_t currentPageStorageSector; // either 0 or 1

    uint8_t writesSinceLastOffsetUpdate_;

    uint32_t pageOffsetBackupStartAddr;
    uint32_t startAddr;
    uint32_t logSpaceInSectors;
    uint32_t sectorSize;


    void WriteLogDataToFlashPageAligned(uint8_t *data, uint16_t size, uint32_t pageAddr);
};



#endif /* FLASH_INC_FLASHLOGHANDLER_HPP_ */
