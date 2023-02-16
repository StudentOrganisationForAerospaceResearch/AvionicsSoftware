#include "SystemStorage.hpp"
#include "FlightTask.hpp"

/**
 * @brief Creates CRC, writes struct and CRC to flash, increases sequence number,
 *        then it erases previous sector
 */
bool SystemStorage::WriteStateToFlash()
{
    //unused
    bool res = true;

    rs_currentInformation.SequenceNumber++;

    uint8_t* data = new uint8_t[12];

    //Store state
    data[0] = (rs_currentInformation.State >> 24) & 0xFF;
    data[1] = (rs_currentInformation.State >> 16) & 0xFF;
    data[2] = (rs_currentInformation.State >> 8) & 0xFF;
    data[3] = (rs_currentInformation.State) & 0xFF;

    //Store sequence number
    data[4] = (rs_currentInformation.SequenceNumber >> 24) & 0xFF;
    data[5] = (rs_currentInformation.SequenceNumber >> 16) & 0xFF;
    data[6] = (rs_currentInformation.SequenceNumber >> 8) & 0xFF;
    data[7] = (rs_currentInformation.SequenceNumber) & 0xFF;
    SOAR_PRINT("Sequence Number: %d\n", rs_currentInformation.SequenceNumber);

    //Calculate and store CRC
    uint32_t checksum = Utils::getCRC32(data, 8);

    data[8] = (checksum >> 24) & 0xFF;
    data[9] = (checksum >> 16) & 0xFF;
    data[10] = (checksum >> 8) & 0xFF;
    data[11] = (checksum) & 0xFF;
    SOAR_PRINT("checksum: %d\n", checksum);

    //Write to relevant sector
    //sector address is not the same as address address but why
    uint32_t addressToWrite = (rs_currentInformation.SequenceNumber % 2);
    W25qxx_WriteSector(data, addressToWrite, 0, 12);

    uint8_t* sector1Data = new uint8_t[12];
    uint8_t* sector2Data = new uint8_t[12];
    W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, 12);
    W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, 12);

    //erase old sector
    uint32_t addressToErase = (rs_currentInformation.SequenceNumber % 2) + 1;
    W25qxx_EraseSector(addressToErase);

    return res;
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

    uint8_t* sector1Data = new uint8_t[12];
    uint8_t* sector2Data = new uint8_t[12];

    //read state sectors
    W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, 12);
    W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, 12);

    //reconstruct and recalculate checksums
    uint32_t sector1ReadChecksum = (sector1Data[8] << 24) | (sector1Data[9] << 16) | (sector1Data[10] << 8) | (sector1Data[11]);
    uint32_t sector2ReadChecksum = (sector2Data[8] << 24) | (sector2Data[9] << 16) | (sector2Data[10] << 8) | (sector2Data[11]);
    SOAR_PRINT("Read Checksum1: %d\n", sector1ReadChecksum);
    SOAR_PRINT("Read Checksum2: %d\n", sector2ReadChecksum);

    uint32_t sector1CalculatedChecksum = Utils::getCRC32(sector1Data, 8);
    uint32_t sector2CalculatedChecksum = Utils::getCRC32(sector2Data, 8);
    SOAR_PRINT("Calculated Checksum1: %d\n", sector1CalculatedChecksum);
    SOAR_PRINT("Calculated Checksum2: %d\n", sector2CalculatedChecksum);

    //reconstruct sequence number
    uint32_t sector1Sequence = sector1Data[4] << 24 | sector1Data[5] << 16 | sector1Data[6] << 8 | sector1Data[7];
    uint32_t sector2Sequence = sector2Data[4] << 24 | sector2Data[5] << 16 | sector2Data[6] << 8 | sector2Data[7];
    SOAR_PRINT("Read Sequence1: %d\n", sector1Sequence);
    SOAR_PRINT("Read Sequence2: %d\n", sector2Sequence);

    uint8_t validSector = 0;

    //find the newest valid sector
    if(sector1ReadChecksum == sector1CalculatedChecksum && sector2ReadChecksum == sector2CalculatedChecksum)
    {
        if(sector1Sequence > sector2Sequence)
        {
            validSector = 1;
            SOAR_PRINT("sector 1 was valid");
        }
        else 
        {
            validSector = 2;
            SOAR_PRINT("sector 2 was valid");
        }
    } 
    else if (sector1ReadChecksum == sector1CalculatedChecksum) 
    {
        validSector = 1;
        SOAR_PRINT("sector 1 was valid");
    } 
    else if (sector2ReadChecksum == sector2CalculatedChecksum)
    {
        validSector = 2;
        SOAR_PRINT("sector 2 was valid");
    } 

    if(validSector == 0) {
        W25qxx_EraseSector(0);
        W25qxx_EraseSector(1);
        res = false;
        SOAR_PRINT("neither sector was valid");
    }

    //write to state struct depending on which sector was deemed valid
    if(validSector == 1) 
    {
        RocketState sector1State = (RocketState) (sector1Data[0] << 24 | sector1Data[1] << 16 | sector1Data[2] << 8 | sector1Data[3]);
        rs_currentInformation = {sector1State, sector1Sequence};
        W25qxx_EraseSector(1);
        res = true;
    }

    if(validSector == 2) 
    {
        RocketState sector2State = (RocketState) (sector2Data[0] << 24 | sector2Data[1] << 16 | sector2Data[2] << 8 | sector2Data[3]);
        rs_currentInformation = {sector2State, sector2Sequence};
        W25qxx_EraseSector(0);
        res = true;
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
    if (res == false)
        rs_currentInformation = {RS_ABORT, 0};
        SOAR_PRINT("readback returned false");
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
    if(cm.GetTaskCommand() == 1) 
    {
        rs_currentInformation.State = (RocketState) (cm.GetDataPointer())[0];
        WriteStateToFlash();
        SOAR_PRINT("state written to flash");
    }
    cm.Reset();
}