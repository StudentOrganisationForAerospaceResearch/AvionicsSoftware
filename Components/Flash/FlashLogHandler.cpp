/*
 * FlashLogHandler.cpp
 *
 *  Created on: May 25, 2024
 *      Author: goada
 */

#include "FlashLogHandler.hpp"
#include <cstring>
#include "w25qxx.hpp"
#include "SPIFlash.hpp"

/**
 * @brief Constructor for FlashLogHandler.
 * @param pageOffsetBackupSectorStart Sector at which page offset backup storage begins. Spans 2 sectors.
 * @param sectorStart Sector at which sensor logging starts.
 * @param logSpaceInSectors Number of sectors that sensor logging space spans
 */
FlashLogHandler::FlashLogHandler(uint32_t pageOffsetBackupSectorStart, uint32_t sectorStart, uint32_t logSpaceInSectors, uint32_t sectorSize) {

    this->pageOffsetBackupStartAddr = pageOffsetBackupSectorStart * SPIFlash::Inst().GetSectorSize();
    this->startAddr = sectorStart * SPIFlash::Inst().GetSectorSize();
    this->logSpaceInSectors = logSpaceInSectors;
    this->sectorSize = sectorSize;

    logbufA = (uint8_t*) soar_malloc(FLASH_HEAP_BUF_SIZE);
    logbufB = (uint8_t*) soar_malloc(FLASH_HEAP_BUF_SIZE);
    currbuf = logbufA;
    offsetWithinBuf = 0;
    SOAR_ASSERT(logbufA && logbufB, "Failed to allocate flash log bufs.\n");
    currentLogPage = startAddr / 256;
    logsInLastSecond = 0;
    lastBaroData = {0};
    lastIMUData = {0};
    lastPTC = {0};
    lastPBBPres = {0};
    currentPageStorageByte = 0;
    currentPageStoragePage = 0;
    currentPageStorageSector = 0;

    RecoverLastWrittenPage(true);


}

/**
 * @brief Log a sensor reading to flash, writing if necessary. Does not erase.
 * @param type The type of log the data represents
 * @param datain The data to log
 * @param size Number of bytes in datain
 */
int FlashLogHandler::AddFlashLog(FLASH_LOG_TYPE type, const uint8_t *datain, uint32_t size) {

    if(currentLogPage >= (logSpaceInSectors*sectorSize)/0xff-2) {
        if(HAL_GetTick() % 500 == 0) {
            SOAR_PRINT("Out of log space!\n");
        }
        return 0;
    }

    logsInLastSecond++;

    uint8_t extraheaderbyte = 0;
    bool useextraheaderbyte = false;

    FLASH_LOG_TYPE thisLogType = type;

    if(thisLogType >= LTYPE_OTHER) {
        SOAR_PRINT("Received unknown log type %d\n",thisLogType);
    }

    useextraheaderbyte = LTYPE_USES_DELTA_ENCODING(thisLogType);

    uint8_t maxbytesAbleToWriteInCurrbuf = FLASH_HEAP_BUF_SIZE - PAGECRCLEN - (offsetWithinBuf); // Num of bytes until end of buf

    uint8_t reducedLog[50];
    memset(reducedLog,0x00,sizeof(reducedLog));

    reducedLog[0] = thisLogType & 0xFF; // First byte of log is LOG TYPE

    int j = 1 + useextraheaderbyte;
    const int32_t* lastData = nullptr;
    size_t numfields = size/sizeof(int32_t);
    if(thisLogType == LTYPE_BAROMETER) {
        lastData = (const int32_t*)(&lastBaroData.pressure_);
    } else if (thisLogType == LTYPE_ACCELGYROMAG) {
        lastData = (const int32_t*)(&lastIMUData.accelX_);
    } else if (thisLogType == LTYPE_PTC) {
        lastData = (const int32_t*)(&lastPTC.pressure);
    } else if (thisLogType == LTYPE_PBB_PRES){
        lastData = (const int32_t*)(&lastPBBPres);
    } else if (thisLogType == LTYPE_GPS) {
        numfields = 8; // some fields of GPS are units
    }


    if(useextraheaderbyte) {
        // Iterate through each int32_t member of the struct.
        // If the member differs from the matching member in the last
        // log of this type by an amount representable by only
        // 1 signed byte, mark it for delta storage to save 3 bytes, reducing writes

        size_t exactsFound = 0;
        for(unsigned int i = 0; i < numfields; i++) {

            int32_t lastValOfThisElement;
            memcpy(&lastValOfThisElement,lastData+i,sizeof(int32_t));

            int32_t currentValOfThisElement;
            memcpy(&currentValOfThisElement, (const int32_t*)(datain)+i,sizeof(int32_t));

            int32_t thisdelta = currentValOfThisElement-lastValOfThisElement;

            if(i < 8 and thisdelta > -128 and thisdelta < 127) {
                extraheaderbyte |= (1 << i);
                int32_t absdelta = thisdelta > 0 ? thisdelta : -thisdelta;
                exactsFound += !absdelta;

                reducedLog[j] = ((int8_t)(absdelta & 0x7f)) * (thisdelta > 0 ? 1 : -1);
                j += 1;
            } else {
                memcpy(reducedLog+j,&currentValOfThisElement,sizeof(int32_t));
                j += sizeof(int32_t);
            }
        }

        // reject exact dupes at same timestamps
        if(exactsFound == numfields) {
            return 0;
        }

        reducedLog[1] = extraheaderbyte;

    } else {
        memcpy(reducedLog+j, datain, size);
        j += size;
    }

    // Num of bytes that should be written in this buf (note when 0, actually 256 bytes could be written)
    uint8_t bytesToWrite = MIN(maxbytesAbleToWriteInCurrbuf == 0 ? 255 : maxbytesAbleToWriteInCurrbuf,j);

    memcpy(currbuf + offsetWithinBuf, reducedLog, bytesToWrite);

    offsetWithinBuf += bytesToWrite;
    bytesToWrite = j - bytesToWrite;

    if(offsetWithinBuf == (uint8_t)((FLASH_HEAP_BUF_SIZE - PAGECRCLEN) & 0xff)) { // just filled the buf

        uint8_t calccrc[PAGECRCLEN] = {0};
        uint16_t th = 0;
        for(unsigned int ci = 0; ci < FLASH_HEAP_BUF_SIZE-PAGECRCLEN; ci++) {
            th += currbuf[ci];
            if(th >= 256) {
                calccrc[0]++;
                th -= 256;
            }

        }
        calccrc[1] = th;
        //calccrc[0] is most sig. byte of checksum, [1] is least sig.
        currbuf[FLASH_HEAP_BUF_SIZE-2] = calccrc[1];
        currbuf[FLASH_HEAP_BUF_SIZE-1] = calccrc[0];

        uint32_t writestarttime;
        if(writebuftimemsg) {
            writestarttime = HAL_GetTick();
        }

        WriteLogDataToFlashPageAligned(currbuf, 256, currentLogPage);

        if(writebuftimemsg) {
            uint32_t writeendtime = HAL_GetTick();

            SOAR_PRINT("Write buffer in %dms\n", writeendtime-writestarttime);
        }
        currbuf = (currbuf == logbufA) ? (logbufB) : (logbufA);
        offsetWithinBuf = 0;
        currentLogPage++;

        // Logs two currentLogPages immediately after each other for offset recovery,
        // the second one acts as a checksum to the first
        uint8_t offsetStorageData[sizeof(currentLogPage)*2];
        memcpy(offsetStorageData,(const uint8_t*)&currentLogPage,sizeof(currentLogPage));
        memcpy(offsetStorageData+sizeof(currentLogPage),(const uint8_t*)&currentLogPage,sizeof(currentLogPage));

        SPIFlash::Inst().Write(pageOffsetBackupStartAddr+currentPageStorageSector*sectorSize+currentPageStoragePage*256 + currentPageStorageByte,
                &(offsetStorageData[0]), sizeof(offsetStorageData));

        currentPageStorageByte+=sizeof(offsetStorageData);

        if(currentPageStorageByte >= 256-sizeof(offsetStorageData)) {
            currentPageStorageByte = 0;
            currentPageStoragePage++;
        }

        if(currentPageStoragePage >= 16) {
            currentPageStorageSector = !currentPageStorageSector;
            currentPageStoragePage = 0;
            SOAR_PRINT("Switching recovery sector...\n");
            // erase the new sector to prepare
            W25qxx_EraseSector(pageOffsetBackupStartAddr/sectorSize+currentPageStorageSector);

        }

    }



    if(bytesToWrite > 0) { // still stuff to write
        memcpy(currbuf + offsetWithinBuf, reducedLog + (j-bytesToWrite), bytesToWrite);
        offsetWithinBuf += bytesToWrite;
    }


    switch(thisLogType) {
    case LTYPE_BAROMETER:
        memcpy(&lastBaroData,datain,size);
        break;
    case LTYPE_ACCELGYROMAG:
        memcpy(&lastIMUData,datain,size);
        break;
    case LTYPE_PTC:
        memcpy(&lastPTC,datain,size);
        break;
    case LTYPE_PBB_PRES:
        memcpy(&lastPBBPres,datain,size);
        break;
    default:
        // ow
        break;
    }

    return 1;
}

/**
 * @brief Erases entire flash chip, including all sensor logs and offset storage. Starts logging from beginning of logging storage.
 */
void FlashLogHandler::ResetLogging() {
    // Erase the chip
    W25qxx_EraseChip();
    currentLogPage = startAddr / 256;
}


/**
 * @brief Prints the current logging page
 */
uint32_t FlashLogHandler::GetCurrentPage() {
    return currentLogPage;
}


/**
 * @brief Returns number of flash logs since last call of this function
 */
uint16_t FlashLogHandler::GetLogBenchmark() {
    uint16_t r = logsInLastSecond;
    logsInLastSecond = 0;
    return r;
}

/**
 * @brief Erases first few sectors in logging space and resets logging to beginning of logging space
 * @param sectorsToErase Number of sectors after sector logging start to erase
 */
void FlashLogHandler::PartialResetLogging(uint16_t sectorsToErase) {

    sectorsToErase = MIN(sectorsToErase,logSpaceInSectors);
    SOAR_PRINT("Erasing %d sectors...\n",sectorsToErase);
    for(int32_t i = 0; i < sectorsToErase; i++) {
        if(!SPIFlash::Inst().Erase(startAddr+i*sectorSize)){
            SOAR_PRINT("Failed Erasing Sectors\n");
        }
    }
    currentLogPage = startAddr/256;

    offsetWithinBuf = 0;
    lastBaroData = {0};
    lastIMUData = {0};
    lastPTC = {0};
    lastPBBPres = {0};
    memset(currbuf,0x00,FLASH_HEAP_BUF_SIZE);
    W25qxx_EraseSector(pageOffsetBackupStartAddr/sectorSize);
    W25qxx_EraseSector(pageOffsetBackupStartAddr/sectorSize+1);
    SOAR_PRINT("Done erasing sectors.\n");
}

/**
 * @brief Search page offset recovery sector and resumes logging at last valid page.
 * @param clear If true, will erase page offset recovery sectors after reading
 * @return true on success
 */
bool FlashLogHandler::RecoverLastWrittenPage(bool clear) {
    { // Recover last written sensor page
        uint32_t highestPage = 0;
        for(uint32_t sec = 0; sec < 2; sec++) {
            for(uint32_t page = 0; page < 16; page++) {
                for(uint32_t byte = 0; byte < 256-sizeof(currentLogPage)*2; byte+=sizeof(currentLogPage)*2) {
                    uint32_t pageread1;
                    uint32_t pageread2;
                    SPIFlash::Inst().Read(pageOffsetBackupStartAddr + sec*sectorSize + page*256 + byte, (uint8_t*)(&pageread1), sizeof(pageread1));
                    SPIFlash::Inst().Read(pageOffsetBackupStartAddr + sec*sectorSize + page*256 + byte + sizeof(currentLogPage), (uint8_t*)(&pageread2), sizeof(pageread2));
                    if(pageread1 == 0xffffffff or pageread2 == 0xffffffff) {
                        continue;
                    }
                    if(pageread1 == pageread2) {
                        if(pageread1 > highestPage) {
                            highestPage = pageread1;
                        }
                    } else {
                        SOAR_PRINT("Encountered corrupt lastpage data\n");
                    }
                }
            }
        }

        SOAR_PRINT("The last written page was %d\n",highestPage);

        currentLogPage = highestPage+1; // continue from after old page
        if(currentLogPage < startAddr/256) {
            currentLogPage = startAddr/256;
        } else if (currentLogPage >= (startAddr+logSpaceInSectors*sectorSize)/0xff-2) {
            currentLogPage = (startAddr+logSpaceInSectors*sectorSize)/0xff-2;
        }

        if(clear) {
        W25qxx_EraseSector(pageOffsetBackupStartAddr/sectorSize);
        W25qxx_EraseSector(pageOffsetBackupStartAddr/sectorSize+1);
        }
    }
    return true;
}

/**
 * @brief Dump first N flash logs through UART
 * @param N Number of logs to dump
 * @param verbose true to print logs in human-readable format, slower
 * @return true on success
 */
bool FlashLogHandler::DumpFirstNLogs(uint32_t N, bool verbose) {
    uint8_t logbuf[128];

    uint32_t readOffsetBytes = startAddr;
    BarometerData lastBaroRead = {0};
    AccelGyroMagnetismData lastIMURead = {0};
    PressureTransducerFlashLogData lastPTCRead = {0};
    PBBPressureFlashLogData lastPBBPresRead = {0};
    const int32_t* lastRead = nullptr;

    SOAR_PRINT("<p>\n"); // used for python interpretation

    do {

        uint8_t logTypeByte = 0;

        W25qxx_ReadByte(&logTypeByte, readOffsetBytes);
        FLASH_LOG_TYPE thistype = (FLASH_LOG_TYPE)logTypeByte;
        readOffsetBytes++;

        uint8_t thisLogSize = 0;

        switch(thistype) {
        case LTYPE_BAROMETER:
            thisLogSize = sizeof(BarometerData);
            lastRead = (const int32_t*)(&lastBaroRead);
            break;
        case LTYPE_ACCELGYROMAG:
            thisLogSize = sizeof(AccelGyroMagnetismData);
            lastRead = (const int32_t*)(&lastIMURead);
            break;
        case LTYPE_PTC:
            thisLogSize = sizeof(PressureTransducerFlashLogData);
            lastRead = (const int32_t*)(&lastPTCRead);
            break;
        case LTYPE_PBB_PRES:
            thisLogSize = sizeof(PBBPressureFlashLogData);
            lastRead = (const int32_t*)(&lastPBBPresRead);
            break;
        case LTYPE_GPS:
            thisLogSize = sizeof(GPSDataFlashLog);
            // no delta storage
            break;
        case LTYPE_MEV_STATE:
            thisLogSize = sizeof(MEVStateFlashLogData);
            // no delta storage
            break;
        default:
            SOAR_PRINT("Encountered unknown log type %d while dumping! <\\p>\n",thistype);
            return false;
        }

        if(verbose) {
            SOAR_PRINT("Log Size %d\nLog Type: %d\n", thisLogSize, thistype);
        } else {
            SOAR_PRINT("S%d T%d\n",thisLogSize,thistype);
        }

        size_t thisNumFields = thisLogSize / sizeof(int32_t);
        bool extraHeaderExists = LTYPE_USES_DELTA_ENCODING(thistype);
        if(extraHeaderExists) {
            thisLogSize++;
        }

        int32_t bytesLeftBeforeCRC = (256-PAGECRCLEN-(readOffsetBytes%256));
        int32_t overflowbytes = thisLogSize - bytesLeftBeforeCRC;
        uint32_t bytesToRead = MIN(thisLogSize,bytesLeftBeforeCRC);
        W25qxx_ReadBytes(logbuf, readOffsetBytes, bytesToRead);
        bool skip = false;

        if(overflowbytes >= 0) {
            skip = true;
            if(overflowbytes > 0) {
                W25qxx_ReadBytes(logbuf + bytesToRead, readOffsetBytes+bytesToRead+PAGECRCLEN, thisLogSize - bytesToRead);
            }
        }

        int32_t* startData = nullptr;
        uint8_t extraheaderbyte = logbuf[0];

        union {
            BarometerData thisBaroRead;
            AccelGyroMagnetismData thisIMURead;
            PressureTransducerFlashLogData thisPTCRead;
            GPSDataFlashLog thisGPSRead;
            PBBPressureFlashLogData thisPBBPresRead;
            MEVStateFlashLogData thisMEVStateRead;
        } thisRead;


        switch(thistype) {
        case LTYPE_BAROMETER:
            startData = (int32_t*)&(thisRead.thisBaroRead);
            break;
        case LTYPE_ACCELGYROMAG:
            startData = (int32_t*)&(thisRead.thisIMURead);
            break;
        case LTYPE_PTC:
            startData = (int32_t*)&(thisRead.thisPTCRead);
            break;
        case LTYPE_GPS:
            startData = (int32_t*)&(thisRead.thisGPSRead);
            break;
        case LTYPE_PBB_PRES:
            startData = (int32_t*)&(thisRead.thisPBBPresRead);
            break;
        case LTYPE_MEV_STATE:
            startData = (int32_t*)&(thisRead.thisMEVStateRead);
        default:
            break;
        }

        int i = 0;
        if(extraHeaderExists) {
            i = 1;
            for(unsigned int field = 0; field < thisNumFields; field++) {
                if(extraheaderbyte & (1<<field)) {
                    int32_t lastFieldValue;
                    memcpy(&lastFieldValue,lastRead+field,sizeof(int32_t));

                    int32_t currentFieldValue = (int8_t)logbuf[i]+lastFieldValue;
                    memcpy(startData+field,&currentFieldValue,sizeof(int32_t));

                    i++;
                } else {
                    memcpy(startData+field,logbuf+i,sizeof(int32_t));
                    i += sizeof(int32_t);
                }
            }
        } else {
            memcpy(startData,logbuf,thisLogSize);
            i=thisLogSize;
        }


        switch(thistype) {
        case LTYPE_BAROMETER:

            if(verbose) {
            SOAR_PRINT("ExtraHeader: %x\nPressure: %d\nTemp: %d\nTime: %d\n\n",
                    extraheaderbyte,thisRead.thisBaroRead.pressure_, thisRead.thisBaroRead.temperature_,
                    thisRead.thisBaroRead.time);
            } else {
                SOAR_PRINT("%d %d %d\n",thisRead.thisBaroRead.pressure_, thisRead.thisBaroRead.temperature_, thisRead.thisBaroRead.time);
            }
            lastBaroRead = thisRead.thisBaroRead;
            break;

        case LTYPE_ACCELGYROMAG:
            if(verbose) {
            SOAR_PRINT(
                    "ExtraHeader: %x\nAccel: %d %d %d\nGyro: %d %d %d\nMagnet: %d %d %d\nTime: %d\n\n",
                    extraheaderbyte, thisRead.thisIMURead.accelX_, thisRead.thisIMURead.accelY_, thisRead.thisIMURead.accelZ_,
                    thisRead.thisIMURead.gyroX_, thisRead.thisIMURead.gyroY_, thisRead.thisIMURead.gyroZ_,
                    thisRead.thisIMURead.magnetoX_, thisRead.thisIMURead.magnetoY_, thisRead.thisIMURead.magnetoZ_,
                    thisRead.thisIMURead.time);
            } else {
                SOAR_PRINT(
                        "%d %d %d %d %d %d %d %d %d %d\n",
                        thisRead.thisIMURead.accelX_, thisRead.thisIMURead.accelY_, thisRead.thisIMURead.accelZ_,
                        thisRead.thisIMURead.gyroX_, thisRead.thisIMURead.gyroY_, thisRead.thisIMURead.gyroZ_,
                        thisRead.thisIMURead.magnetoX_, thisRead.thisIMURead.magnetoY_, thisRead.thisIMURead.magnetoZ_,
                        thisRead.thisIMURead.time);
            }
            lastIMURead = thisRead.thisIMURead;
            break;

        case LTYPE_PTC:

            if(verbose) {
                SOAR_PRINT("ExtraHeader: %x\nPTC Pressure: %d, Time: %d\n\n",extraheaderbyte,thisRead.thisPTCRead.pressure,thisRead.thisPTCRead.time);
            } else {
                SOAR_PRINT("%d %d\n",thisRead.thisPTCRead.pressure,thisRead.thisPTCRead.time);

            }
            lastPTCRead = thisRead.thisPTCRead;
            break;

        case LTYPE_GPS:

            if(verbose) {
                SOAR_PRINT("Time: %d\nLat: %d deg, %d mins\nLong: %d deg, %d mins\nAntenna Alt: %d %c\nGeoid Alt: %d %c\nTotal Alt: %d %c\n\n",
                    thisRead.thisGPSRead.time_, thisRead.thisGPSRead.latitude_.degrees_, thisRead.thisGPSRead.latitude_.minutes_,
                    thisRead.thisGPSRead.longitude_.degrees_, thisRead.thisGPSRead.longitude_.minutes_,
                    thisRead.thisGPSRead.antennaAltitude_.altitude_, thisRead.thisGPSRead.antennaAltitude_.unit_,
                    thisRead.thisGPSRead.geoidAltitude_.altitude_,thisRead.thisGPSRead.geoidAltitude_.unit_,
                    thisRead.thisGPSRead.totalAltitude_.altitude_,thisRead.thisGPSRead.totalAltitude_.unit_);
            } else {
                SOAR_PRINT("%d %d %d %d %d %d %c %d %c %d %c\n",
                        thisRead.thisGPSRead.time_, thisRead.thisGPSRead.latitude_.degrees_, thisRead.thisGPSRead.latitude_.minutes_,
                        thisRead.thisGPSRead.longitude_.degrees_, thisRead.thisGPSRead.longitude_.minutes_,
                        thisRead.thisGPSRead.antennaAltitude_.altitude_, thisRead.thisGPSRead.antennaAltitude_.unit_,
                        thisRead.thisGPSRead.geoidAltitude_.altitude_,thisRead.thisGPSRead.geoidAltitude_.unit_,
                        thisRead.thisGPSRead.totalAltitude_.altitude_,thisRead.thisGPSRead.totalAltitude_.unit_);

            }
            break;

        case LTYPE_PBB_PRES:

            if(verbose) {
            SOAR_PRINT("PBB PRES!!! IBPressure: %d\n, LOWERPVPRESSURE: %d\n Time: %d\n\n",
                    thisRead.thisPBBPresRead.ib_pressure, thisRead.thisPBBPresRead.lower_pv_pressure, thisRead.thisPBBPresRead.time);
            } else {
                SOAR_PRINT("%d %d %d\n",
                        thisRead.thisPBBPresRead.ib_pressure, thisRead.thisPBBPresRead.lower_pv_pressure, thisRead.thisPBBPresRead.time);

            }
            lastPBBPresRead = thisRead.thisPBBPresRead;
            break;


        case LTYPE_MEV_STATE:
            if(verbose) {
                SOAR_PRINT("MEV State! Open: %d\nTime:%d\n",thisRead.thisMEVStateRead.stateandtime>>31,thisRead.thisMEVStateRead.stateandtime&0x7fffffff);
            } else {
                SOAR_PRINT("%d %d\n", (thisRead.thisMEVStateRead.stateandtime>>31)&1,thisRead.thisMEVStateRead.stateandtime&0x7fffffff);
            }


            break;

        default:

            SOAR_PRINT("Unknown log type\n");
        }


        if(skip) {
            if(!(readOffsetBytes%256 + i >= 256-PAGECRCLEN)) {
                skip = false;
            } else {
                uint8_t crc[PAGECRCLEN];
                W25qxx_ReadBytes(crc, readOffsetBytes/256*256+256-PAGECRCLEN, PAGECRCLEN);

                uint8_t calccrc[PAGECRCLEN] = {0};
                uint8_t lastpage[FLASH_HEAP_BUF_SIZE-PAGECRCLEN];
                W25qxx_ReadBytes(lastpage, readOffsetBytes/256*256, FLASH_HEAP_BUF_SIZE-PAGECRCLEN);

                uint16_t th = 0;
                for(unsigned int ci = 0; ci < FLASH_HEAP_BUF_SIZE-PAGECRCLEN; ci++) {
                    th += lastpage[ci];
                    if(th >= 256) {
                        calccrc[0]++;
                        th -= 256;
                    }

                }

                calccrc[1] = th;
                if(crc [1] == calccrc[0] && crc[0] == calccrc[1]) {
                    // passed crc
                    if(verbose) {
                        SOAR_PRINT("Passed CRC\n");
                    }
                } else {
                    SOAR_PRINT("Failed flash CRC at page %d (%x %x, expected %x %x)\n<\\p>\n", readOffsetBytes/256, crc[0], crc[1], calccrc[0], calccrc[1]);
                    return false;
                }
            }
        }
        readOffsetBytes += i + PAGECRCLEN*skip;
        N--;

    } while (N > 0);

    SOAR_PRINT("<\\p>\n"); // end token for python interpretation

    return true;
}

/**
 * @brief Simple wrapper for writing page-aligned data to flash
 */
void FlashLogHandler::WriteLogDataToFlashPageAligned(uint8_t *data, uint16_t size, uint32_t pageAddr) {
    W25qxx_WritePage(data, pageAddr, 0, size);
}

/**
 * @brief Toggle whether a time measurement is printed after each logging page write
 * @return Returns value after toggle
 */
bool FlashLogHandler::ToggleMessageOnWrite() {
    return (writebuftimemsg = !writebuftimemsg);
}
