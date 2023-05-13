#include "SystemStorage.hpp"
#include "FlightTask.hpp"
#include "Data.h"
#include <cstring>     // Support for memcpy

/**
 * @brief Default constructor for SystemStorage, initializes flash struct
 */
SystemStorage::SystemStorage()
{
    W25qxx_Init();

    // Read from flash to populate state struct
    bool res = ReadStateFromFlash();
    if (res == false) {
        sys_currentState = { RS_ABORT, 0, 0 };
        //should probably erase all data sectors
        SOAR_PRINT("readback returned false");
    }
    Command cmd(FLASH_RESPONSE, (uint16_t)sys_currentState.rocketState);
    FlightTask::Inst().GetEventQueue()->Send(cmd);
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void SystemStorage::HandleCommand(Command& cm)
{
    SOAR_ASSERT(w25qxx.UniqID[0] != 0, "Flash command received before flash was initialized");

    switch (cm.GetCommand()) {
    case TASK_SPECIFIC_COMMAND: {
        if (cm.GetTaskCommand() == WRITE_STATE_TO_FLASH)
        {
            sys_currentState.rocketState = (RocketState)(cm.GetDataPointer()[0]);
            WriteStateToFlash();
            SOAR_PRINT("state written to flash\n");
        }
        else if (cm.GetTaskCommand() == DUMP_FLASH_DATA)
        {
            ReadDataFromFlash();
        }
        else if (cm.GetTaskCommand() == ERASE_ALL_FLASH)
        {
            W25qxx_EraseChip();
            sys_currentState.data_offset = 0;
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


/**
 * @brief Creates CRC, writes struct and CRC to flash, increases sequence number,
 *        then it erases previous sector
 */
void SystemStorage::WriteStateToFlash()
{
    sys_currentState.sequenceNum++;
    //SOAR_PRINT("sequence number: %d\n", sys_currentState.sequenceNum);

    uint8_t data[sizeof(StateInformation) + sizeof(uint32_t)];

    memcpy(data, &sys_currentState, sizeof(StateInformation));
    //SOAR_PRINT("rocketState: %d\n", sys_currentState.rocketState);

    //Calculate and store CRC
    uint32_t checksum = Utils::getCRC32(data, sizeof(StateInformation));

    memcpy(data + sizeof(StateInformation), &checksum, sizeof(uint32_t));
    //SOAR_PRINT("checksum: %d\n", checksum);

    //Write to relevant sector
    //sector address is not the same as address address
    uint32_t addressToWrite = (sys_currentState.sequenceNum % 2);
    W25qxx_WriteSector(data, addressToWrite, 0, sizeof(StateInformation) + sizeof(uint32_t));

    //erase old sector
    uint32_t addressToErase = ((sys_currentState.sequenceNum + 1) % 2);
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

    StateInformation* sector1;
    StateInformation* sector2;

    uint32_t sector1CalculatedChecksum;
    uint32_t sector2CalculatedChecksum;

    //read state sectors
    W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, sizeof(StateInformation) + sizeof(uint32_t));
    W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, sizeof(StateInformation) + sizeof(uint32_t));
	sector1 = (StateInformation*)sector1Data;
    sector2 = (StateInformation*)sector2Data;

    //get the checksums
	uint32_t sector1ReadChecksum = *(uint32_t*)(sector1Data + sizeof(StateInformation));
	uint32_t sector2ReadChecksum = *(uint32_t*)(sector2Data + sizeof(StateInformation));

    sector1CalculatedChecksum = Utils::getCRC32(sector1Data, sizeof(StateInformation));
    sector2CalculatedChecksum = Utils::getCRC32(sector2Data, sizeof(StateInformation));

    uint8_t validSector = 0;

    //find the newest valid sector
    if(sector1ReadChecksum == sector1CalculatedChecksum && sector2ReadChecksum == sector2CalculatedChecksum)
    {
        if(sector1->sequenceNum > sector2->sequenceNum)
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
        sys_currentState = *sector1;
        W25qxx_EraseSector(1);
        res = true;
    }
	if(validSector == 2) 
    {
        sys_currentState = *sector2;
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
    uint32_t sectorAddressToWrite = (sys_currentState.data_offset + INITIAL_SENSOR_FLASH_OFFSET) / w25qxx.SectorSize;
    uint32_t sectorOffsetToWrite = (sys_currentState.data_offset + INITIAL_SENSOR_FLASH_OFFSET) % w25qxx.SectorSize;

    uint8_t buff[size + 1];

    buff[0] = (uint8_t)(size & 0xff);
    memcpy(buff + 1, data, size);

    W25qxx_WriteSector(buff, sectorAddressToWrite, sectorOffsetToWrite, size + 1);

    sys_currentState.data_offset = sys_currentState.data_offset + size + 1; //address is in bytes
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

    uint8_t length;

    for(unsigned int i = 0; i < sys_currentState.data_offset + INITIAL_SENSOR_FLASH_OFFSET; i++) {
        W25qxx_ReadByte(&length, INITIAL_SENSOR_FLASH_OFFSET + i);

        if(length == sizeof(AccelGyroMagnetismData)) {
            uint8_t dataRead[sizeof(AccelGyroMagnetismData)];
            W25qxx_ReadBytes(dataRead, INITIAL_SENSOR_FLASH_OFFSET + i + 1, sizeof(AccelGyroMagnetismData));
            AccelGyroMagnetismData* IMURead = (AccelGyroMagnetismData*)dataRead;
            SOAR_PRINT("%03d %08d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\n", 
                    length, IMURead->time, IMURead->accelX_, IMURead->accelY_, IMURead->accelZ_,
                    IMURead->gyroX_, IMURead->gyroY_, IMURead->gyroZ_, IMURead->magnetoX_,
                    IMURead->magnetoY_, IMURead->magnetoZ_);
        }
        else if(length == sizeof(BarometerData)) {
            uint8_t dataRead[sizeof(BarometerData)];
            W25qxx_ReadBytes(dataRead, INITIAL_SENSOR_FLASH_OFFSET + i + 1, sizeof(BarometerData));
            BarometerData* baroRead = (BarometerData*)dataRead;
            SOAR_PRINT("%3d %08d   %04d   %04d\n",
                    length, baroRead->time, baroRead->pressure_, baroRead->temperature_);
        } else {
            SOAR_PRINT("Unknown length, readback brokedown: %d\n", length);
        }
        i = i + length;
    }
    return res;
}