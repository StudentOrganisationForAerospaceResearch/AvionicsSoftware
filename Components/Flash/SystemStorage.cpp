#include "SystemStorage.hpp"
#include "FlightTask.hpp"
#include "Data.h"
#include <cstring>     // Support for memcpy

/**
 * @brief Creates CRC, writes struct and CRC to flash, increases sequence number,
 *        then it erases previous sector
 */
void SystemStorage::WriteStateToFlash()
{
    rs_currentInformation.SequenceNumber++;
    //SOAR_PRINT("sequence number: %d\n", rs_currentInformation.SequenceNumber);

    uint8_t data[sizeof(StateInformation) + sizeof(uint32_t)];

    memcpy(data, &rs_currentInformation, sizeof(StateInformation));
    //SOAR_PRINT("State: %d\n", rs_currentInformation.State);

    //Calculate and store CRC
    uint32_t checksum = Utils::getCRC32(data, sizeof(StateInformation));

    memcpy(data + sizeof(StateInformation), &checksum, sizeof(uint32_t));
    //SOAR_PRINT("checksum: %d\n", checksum);

    //Write to relevant sector
    //sector address is not the same as address address
    uint32_t addressToWrite = (rs_currentInformation.SequenceNumber % 2);
    W25qxx_WriteSector(data, addressToWrite, 0, sizeof(StateInformation) + sizeof(uint32_t));

    //erase old sector
    uint32_t addressToErase = ((rs_currentInformation.SequenceNumber + 1) % 2);
    W25qxx_EraseSector(addressToErase);
}

/**
 * @brief reads state data from flash and checks CRC:
 *        if both are correct, writes the one with the highest nuber to its struct and returns TRUE.
 *        If only one is right, writes that one to its struct and returns TRUE. 
 *        If neither are right, it returns FALSE
 */
bool SystemStorage::ReadStateFromFlash()
{
    bool res = true;

    uint8_t sector1Data[sizeof(StateInformation) + sizeof(uint32_t)];
    uint8_t sector2Data[sizeof(StateInformation) + sizeof(uint32_t)];

    StateInformation sector1;
    StateInformation sector2;

    uint32_t sector1ReadChecksum;
    uint32_t sector2ReadChecksum;

    uint32_t sector1CalculatedChecksum;
    uint32_t sector2CalculatedChecksum;

    //read state sectors
    W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, sizeof(StateInformation) + sizeof(uint32_t));
    W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, sizeof(StateInformation) + sizeof(uint32_t));

    memcpy(&sector1, sector1Data, sizeof(StateInformation));
    memcpy(&sector2, sector2Data, sizeof(StateInformation));
    //SOAR_PRINT("Read State1: %d\n", sector1.State);
    //SOAR_PRINT("Read State2: %d\n", sector2.State);

    memcpy(&sector1ReadChecksum, sector1Data + sizeof(StateInformation), sizeof(uint32_t));
    memcpy(&sector2ReadChecksum, sector2Data + sizeof(StateInformation), sizeof(uint32_t));
    //SOAR_PRINT("Read Checksum1: %d\n", sector1ReadChecksum);
    //SOAR_PRINT("Read Checksum2: %d\n", sector2ReadChecksum);

    sector1CalculatedChecksum = Utils::getCRC32(sector1Data, sizeof(StateInformation));
    sector2CalculatedChecksum = Utils::getCRC32(sector2Data, sizeof(StateInformation));
    //SOAR_PRINT("Calculated Checksum1: %d\n", sector1CalculatedChecksum);
    //SOAR_PRINT("Calculated Checksum2: %d\n", sector2CalculatedChecksum);

    //SOAR_PRINT("Read Sequence1: %d\n", sector1.SequenceNumber);
    //SOAR_PRINT("Read Sequence2: %d\n", sector2.SequenceNumber);

    uint8_t validSector = 0;

    //find the newest valid sector
    if(sector1ReadChecksum == sector1CalculatedChecksum && sector2ReadChecksum == sector2CalculatedChecksum)
    {
        if(sector1.SequenceNumber > sector2.SequenceNumber)
        {
            validSector = 1;
            SOAR_PRINT("sector 1 was valid\n");
        }
        else 
        {
            validSector = 2;
            SOAR_PRINT("sector 2 was valid\n");
        }
    } 
    else if (sector1ReadChecksum == sector1CalculatedChecksum) 
    {
        validSector = 1;
        SOAR_PRINT("sector 1 was valid\n");
    } 
    else if (sector2ReadChecksum == sector2CalculatedChecksum)
    {
        validSector = 2;
        SOAR_PRINT("sector 2 was valid\n");
    } 

    if(validSector == 0) {
        W25qxx_EraseSector(0);
        W25qxx_EraseSector(1);
        res = false;
        SOAR_PRINT("neither sector was valid\n");
    }

    //write to state struct depending on which sector was deemed valid
    if(validSector == 1) 
    {
        rs_currentInformation = sector1;
        W25qxx_EraseSector(1);
        res = true;
    }

    if(validSector == 2) 
    {
        rs_currentInformation = sector2;
        W25qxx_EraseSector(0);
        res = true;
    }

    return res;
}

/**
 * @brief writes data to flash with the size of the data written as the header, increases offset by size + 1 to account for size, currently only handles size < 255
 */
void SystemStorage::WriteDataToFlash(uint8_t* data, uint16_t size)
{
    //Write to relevant sector
    uint32_t addressToWrite = rs_currentInformation.data_offset + INITIAL_SENSOR_FLASH_OFFSET;

    W25qxx_WriteByte((uint8_t)(size & 0xff), addressToWrite);

    for(uint32_t i = 0; i < size; i++) {
        W25qxx_WriteByte(data[i], addressToWrite + i + 1);
    }

    rs_currentInformation.data_offset = rs_currentInformation.data_offset + size + 1; //address is in bytes
    WriteStateToFlash();
}

/**
 * @brief reads all data and prints it through UART up until offset read from struct
 *        currently unimplemented
 */
bool SystemStorage::ReadDataFromFlash()
{
    //unused
    bool res = true;

    uint8_t length

    for(i = 0; i < rs_currentInformation.data_offset + INITIAL_SENSOR_FLASH_OFFSET; i++) {
        W25qxx_ReadByte(length, INITIAL_SENSOR_FLASH_OFFSET + i);

        if(length == sizeof(AccelGyroMagnetismData)) {
            uint8_t dataRead[sizeof(AccelGyroMagnetismData)];
            W25qxx_ReadBytes(dataRead, INITIAL_SENSOR_FLASH_OFFSET + i + 1, sizeof(AccelGyroMagnetismData));
            AccelGyroMagnetismData* IMURead = dataRead;
            SOAR_PRINT("%d   %d   %d   %d   %d   %d   %d   %d   %d   %d", 
                    IMURead->time, IMURead->accelX_, IMURead->accelY_, IMURead->accelZ_,
                    IMURead->gyroX_, IMURead->gyroY_, IMURead->gyroZ_, IMURead->magnetoX_,
                    IMURead->magnetoY_, IMURead->magnetoZ_);
        }
        else if(length == sizeof(BarometerData)) {
            uint8_t dataRead[sizeof(BarometerData)];
            W25qxx_ReadBytes(dataRead, INITIAL_SENSOR_FLASH_OFFSET + i + 1, sizeof(BarometerData));
            BarometerData* baroRead = dataRead;
            SOAR_PRINT("%d   %d   %d",
                    baroRead->time, baroRead->pressure_, baroRead->temperature_);
        } else {
            SOAR_PRINT("Unknown length, readback brokedown");
        }
    }
    return res;
}

/**
 * @brief Default constructor for SystemStorage, initializes flash struct
 */
SystemStorage::SystemStorage()
{
    W25qxx_Init();
    
    //read from flash to populate state struct
    bool res = ReadStateFromFlash();
    if (res == false) {
        rs_currentInformation = {RS_ABORT, 0};
        //should probably erase all data sectors
        SOAR_PRINT("readback returned false");
    }
    Command cmd(FLASH_RESPONSE, (uint16_t) rs_currentInformation.State);
    FlightTask::Inst().GetEventQueue()->Send(cmd);
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void SystemStorage::HandleCommand(Command& cm)
{
    SOAR_ASSERT(w25qxx.UniqID[0] != 0, "Flash command received before flash was initialized");
    switch(cm.GetCommand()) {
        case TASK_SPECIFIC_COMMAND: {
            if(cm.GetTaskCommand() == WRITE_STATE_TO_FLASH) 
            {
                rs_currentInformation.State = (RocketState) (cm.GetDataPointer()[0]);
                WriteStateToFlash();
                SOAR_PRINT("state written to flash\n");
            }
            else if(cm.GetTaskCommand() == DUMP_FLASH_DATA) 
            {
                ReadDataFromFlash();
            }
            else if(cm.GetTaskCommand() == ERASE_ALL_FLASH)
            {
                W25qxx_EraseChip();
            }
            break;
        }
        case DATA_COMMAND: {
            WriteDataToFlash(cm.GetDataPointer(), cm.GetDataSize());
            break;
        }
        default:
            break;
    }
    cm.Reset();
}