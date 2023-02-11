#include "SystemStorage.hpp"
#include "FlightTask.hpp"

/**
 * @brief Creates CRC, writes struct and CRC to flash, increases sequence number, then it erases previous sector
 */
bool SystemStorage::WriteStateToFlash()
{
    bool res = true;

    uint8_t* data = new uint8_t[12];

    data[0] = (rs_currentInformation.State >> 24) & 0xFF;
    data[1] = (rs_currentInformation.State >> 16) & 0xFF;
    data[2] = (rs_currentInformation.State >> 8) & 0xFF;
    data[3] = (rs_currentInformation.State) & 0xFF;

    data[4] = (rs_currentInformation.SequenceNumber >> 24) & 0xFF;
    data[5] = (rs_currentInformation.SequenceNumber >> 16) & 0xFF;
    data[6] = (rs_currentInformation.SequenceNumber >> 8) & 0xFF;
    data[7] = (rs_currentInformation.SequenceNumber) & 0xFF;

    uint32_t checksum = Utils::getCRC32(data, 8);

    data[8] = (checksum >> 24) & 0xFF;
    data[9] = (checksum >> 16) & 0xFF;
    data[10] = (checksum >> 8) & 0xFF;
    data[11] = (checksum) & 0xFF;

    uint32_t addressToWrite = w25qxx.SectorSize * (rs_currentInformation.SequenceNumber % 2);

    W25qxx_WriteSector(data, addressToWrite, 0, 12);

    rs_currentInformation.SequenceNumber++;

    uint32_t addressToErase = w25qxx.SectorSize * (rs_currentInformation.SequenceNumber % 2);

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

    W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, 12);
    W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, 12);

    uint32_t sector1ReadChecksum = sector1Data[8] << 24 & sector1Data[9] << 16 & sector1Data[10] << 8 & sector1Data[11];
    uint32_t sector2ReadChecksum = sector2Data[8] << 24 & sector2Data[9] << 16 & sector2Data[10] << 8 & sector2Data[11];

    uint32_t sector1CalculatedChecksum = Utils::getCRC32(sector1Data, 8);
    uint32_t sector2CalculatedChecksum = Utils::getCRC32(sector1Data, 8);

    uint32_t sector1Sequence = sector1Data[4] << 24 & sector1Data[5] << 16 & sector1Data[6] << 8 & sector1Data[7];
    uint32_t sector2Sequence = sector2Data[4] << 24 & sector2Data[5] << 16 & sector2Data[6] << 8 & sector2Data[7];

    uint8_t validSector = 0;

    if(sector1ReadChecksum == sector1CalculatedChecksum && sector2ReadChecksum == sector2CalculatedChecksum)
    {
        if(sector1Sequence > sector2Sequence)
        {
            validSector = 1;
        }
        else 
        {
            validSector = 2;
        }
    } 
    else if (sector1ReadChecksum == sector1CalculatedChecksum) 
    {
        validSector = 1;
    } 
    else if (sector2ReadChecksum == sector2CalculatedChecksum)
    {
        validSector = 2;
    } 

    if(validSector == 0) {
        W25qxx_EraseSector(w25qxx.SectorSize * 0);
        W25qxx_EraseSector(w25qxx.SectorSize * 1);
        res = false;
    }

    if(validSector == 1) 
    {
        RocketState sector1State = (RocketState) (sector1Data[0] << 24 & sector1Data[1] << 16 & sector1Data[2] << 8 & sector1Data[3]);
        rs_currentInformation = {sector1State, sector1Sequence};
        W25qxx_EraseSector(w25qxx.SectorSize * 1);
        res = true;
    }

    if(validSector == 2) 
    {
        RocketState sector2State = (RocketState) (sector2Data[0] << 24 & sector2Data[1] << 16 & sector2Data[2] << 8 & sector2Data[3]);
        rs_currentInformation = {sector2State, sector2Sequence};
        W25qxx_EraseSector(w25qxx.SectorSize * 0);
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
    // read from flash to find newest sequence number and initialize rs_currentInformation to that
    bool res = ReadStateFromFlash();
    if (res == false)
        rs_currentInformation = {RS_ABORT, 0};
    
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