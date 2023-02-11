#include "SystemStorage.hpp"

/**
 * @brief Default constructor for SystemStorage, initializes flash struct
 */
SystemStorage::SystemStorage()
{
    W25qxx_Init();
    // read from flash to find newest sequence number and initialize rs_currentInformation to that
    rs_currentInformation = {RS_ABORT, 0};
}

/**
 * @brief Creates CRC, writes struct and CRC to flash, increases sequence number, then it erases previous sector
 */
bool SystemStorage::writeStateToFlash()
{
    bool res = true;

    uint8_t* data = new uint8_t[12];

    data[0] = (rs_currentInformation.State) & 0xFF;
    data[1] = (rs_currentInformation.State >> 8) & 0xFF;
    data[2] = (rs_currentInformation.State >> 16) & 0xFF;
    data[3] = (rs_currentInformation.State >> 24) & 0xFF;

    data[4] = (rs_currentInformation.SequenceNumber) & 0xFF;
    data[5] = (rs_currentInformation.SequenceNumber >> 8) & 0xFF;
    data[6] = (rs_currentInformation.SequenceNumber >> 16) & 0xFF;
    data[7] = (rs_currentInformation.SequenceNumber >> 24) & 0xFF;

    uint32_t checksum = Utils::getCRC32(data, 8);

    data[8] = (checksum) & 0xFF;
    data[9] = (checksum >> 8) & 0xFF;
    data[10] = (checksum >> 16) & 0xFF;
    data[11] = (checksum >> 24) & 0xFF;

    uint32_t addressToWrite = w25qxx.SectorSize * (rs_currentInformation.SequenceNumber % 2);

    W25qxx_WriteSector(data, addressToWrite, 0, 12);

    rs_currentInformation.SequenceNumber++;

    uint32_t addressToErase = w25qxx.SectorSize * (rs_currentInformation.SequenceNumber % 2);

    W25qxx_EraseSector(addressToErase);

    return res;
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
        writeStateToFlash();
    }
    cm.Reset();
    SOAR_PRINT("state written to flash");
}