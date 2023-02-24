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

    uint8_t* data = new uint8_t[16];

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

    //Store data offset
    data[8] = (si_currentInformation.offset >> 24) & 0xFF;
    data[9] = (si_currentInformation.offset >> 16) & 0xFF;
    data[10] = (si_currentInformation.offset >> 8) & 0xFF;
    data[11] = (si_currentInformation.offset) & 0xFF;

    //Calculate and store CRC
    uint32_t checksum = Utils::getCRC32(data, 12);

    data[12] = (checksum >> 24) & 0xFF;
    data[13] = (checksum >> 16) & 0xFF;
    data[14] = (checksum >> 8) & 0xFF;
    data[15] = (checksum) & 0xFF;
    SOAR_PRINT("checksum: %d\n", checksum);

    //Write to relevant sector
    //sector address is not the same as address address but why
    uint32_t addressToWrite = (rs_currentInformation.SequenceNumber % 2);
    W25qxx_WriteSector(data, addressToWrite, 0, 16);

    //for debugging
    //uint8_t* sector1Data = new uint8_t[16];
    //uint8_t* sector2Data = new uint8_t[16];
    //W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, 16);
    //W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, 16);

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

    uint8_t* sector1Data = new uint8_t[16];
    uint8_t* sector2Data = new uint8_t[16];

    //read state sectors
    W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, 16);
    W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, 16);

    //reconstruct and recalculate checksums
    uint32_t sector1ReadChecksum = (sector1Data[12] << 24) | (sector1Data[13] << 16) | (sector1Data[14] << 8) | (sector1Data[15]);
    uint32_t sector2ReadChecksum = (sector2Data[12] << 24) | (sector2Data[13] << 16) | (sector2Data[14] << 8) | (sector2Data[15]);
    //SOAR_PRINT("Read Checksum1: %d\n", sector1ReadChecksum);
    //SOAR_PRINT("Read Checksum2: %d\n", sector2ReadChecksum);

    uint32_t sector1CalculatedChecksum = Utils::getCRC32(sector1Data, 12);
    uint32_t sector2CalculatedChecksum = Utils::getCRC32(sector2Data, 12);
    //SOAR_PRINT("Calculated Checksum1: %d\n", sector1CalculatedChecksum);
    //SOAR_PRINT("Calculated Checksum2: %d\n", sector2CalculatedChecksum);

    //reconstruct sequence number
    uint32_t sector1Sequence = sector1Data[4] << 24 | sector1Data[5] << 16 | sector1Data[6] << 8 | sector1Data[7];
    uint32_t sector2Sequence = sector2Data[4] << 24 | sector2Data[5] << 16 | sector2Data[6] << 8 | sector2Data[7];
    //SOAR_PRINT("Read Sequence1: %d\n", sector1Sequence);
    //SOAR_PRINT("Read Sequence2: %d\n", sector2Sequence);

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
        uint32_t offset = (sector1Data[8] << 24 | sector1Data[9] << 16 | sector1Data[10] << 8 | sector1Data[11]);
        si_currentInformation = {offset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        W25qxx_EraseSector(1);
        res = true;
    }

    if(validSector == 2) 
    {
        RocketState sector2State = (RocketState) (sector2Data[0] << 24 | sector2Data[1] << 16 | sector2Data[2] << 8 | sector2Data[3]);
        rs_currentInformation = {sector2State, sector2Sequence};
        uint32_t offset = (sector2Data[8] << 24 | sector2Data[9] << 16 | sector2Data[10] << 8 | sector2Data[11]);
        si_currentInformation = {offset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        W25qxx_EraseSector(0);
        res = true;
    }

    return res;
}

/**
 * @brief Creates CRC, writes sensor info struct and CRC to flash, increases offset
 */
bool SystemStorage::writeSensorInfoToFlash()
{
    //unused
    bool res = true;

    si_currentInformation.offset = si_currentInformation.offset + 64; //address is in bytes

    uint8_t* data = new uint8_t[64];

    uint32_t time = TICKS_TO_MS(xTaskGetTickCount()) / 1000;

    //Store beginning of packet
    data[0] = 0b10100101
    data[1] = 0b10100101;
    data[2] = 0b00000000;
    data[3] = 0b11111111;   

    //Store time
    data[4] = (time >> 24) & 0xFF;
    data[5] = (time >> 16) & 0xFF;
    data[6] = (time >> 8) & 0xFF;
    data[7] = (time) & 0xFF;

    //Store accelX_
    data[8] = (si_currentInformation.accelX_ >> 24) & 0xFF;
    data[9] = (si_currentInformation.accelX_ >> 16) & 0xFF;
    data[10] = (si_currentInformation.accelX_ >> 8) & 0xFF;
    data[11] = (si_currentInformation.accelX_) & 0xFF;

    //Store accelY_
    data[12] = (si_currentInformation.accelY_ >> 24) & 0xFF;
    data[13] = (si_currentInformation.accelY_ >> 16) & 0xFF;
    data[14] = (si_currentInformation.accelY_ >> 8) & 0xFF;
    data[15] = (si_currentInformation.accelY_) & 0xFF;

    //Store accelZ_
    data[16] = (si_currentInformation.accelZ_ >> 24) & 0xFF;
    data[17] = (si_currentInformation.accelZ_ >> 16) & 0xFF;
    data[18] = (si_currentInformation.accelZ_ >> 8) & 0xFF;
    data[19] = (si_currentInformation.accelZ_) & 0xFF;

    //Store gyroX_
    data[20] = (si_currentInformation.gyroX_ >> 24) & 0xFF;
    data[21] = (si_currentInformation.gyroX_ >> 16) & 0xFF;
    data[22] = (si_currentInformation.gyroX_ >> 8) & 0xFF;
    data[23] = (si_currentInformation.gyroX_) & 0xFF;

    //Store gyroY_
    data[24] = (si_currentInformation.gyroY_ >> 24) & 0xFF;
    data[25] = (si_currentInformation.gyroY_ >> 16) & 0xFF;
    data[26] = (si_currentInformation.gyroY_ >> 8) & 0xFF;
    data[27] = (si_currentInformation.gyroY_) & 0xFF;

    //Store gyroZ_
    data[28] = (si_currentInformation.gyroZ_ >> 24) & 0xFF;
    data[29] = (si_currentInformation.gyroZ_ >> 16) & 0xFF;
    data[30] = (si_currentInformation.gyroZ_ >> 8) & 0xFF;
    data[31] = (si_currentInformation.gyroZ_) & 0xFF;

    //Store magnetoX_
    data[32] = (si_currentInformation.magnetoX_ >> 24) & 0xFF;
    data[33] = (si_currentInformation.magnetoX_ >> 16) & 0xFF;
    data[34] = (si_currentInformation.magnetoX_ >> 8) & 0xFF;
    data[35] = (si_currentInformation.magnetoX_) & 0xFF;

    //Store magnetoY_
    data[36] = (si_currentInformation.magnetoY_ >> 24) & 0xFF;
    data[37] = (si_currentInformation.magnetoY_ >> 16) & 0xFF;
    data[38] = (si_currentInformation.magnetoY_ >> 8) & 0xFF;
    data[39] = (si_currentInformation.magnetoY_) & 0xFF;

    //Store magnetoZ_
    data[40] = (si_currentInformation.magnetoZ_ >> 24) & 0xFF;
    data[41] = (si_currentInformation.magnetoZ_ >> 16) & 0xFF;
    data[42] = (si_currentInformation.magnetoZ_ >> 8) & 0xFF;
    data[43] = (si_currentInformation.magnetoZ_) & 0xFF;

    //Store pressure_
    data[44] = (si_currentInformation.pressure_ >> 24) & 0xFF;
    data[45] = (si_currentInformation.pressure_ >> 16) & 0xFF;
    data[46] = (si_currentInformation.pressure_ >> 8) & 0xFF;
    data[47] = (si_currentInformation.pressure_) & 0xFF;

    //Store temperature_
    data[48] = (si_currentInformation.temperature_ >> 24) & 0xFF;
    data[49] = (si_currentInformation.temperature_ >> 16) & 0xFF;
    data[50] = (si_currentInformation.temperature_ >> 8) & 0xFF;
    data[51] = (si_currentInformation.temperature_) & 0xFF;

    //Store offset
    data[52] = (si_currentInformation.offset >> 24) & 0xFF;
    data[53] = (si_currentInformation.offset >> 16) & 0xFF;
    data[54] = (si_currentInformation.offset >> 8) & 0xFF;
    data[55] = (si_currentInformation.offset) & 0xFF;

    //Calculate and store CRC
    uint32_t checksum = Utils::getCRC32(data, 56);

    data[56] = (checksum >> 24) & 0xFF;
    data[57] = (checksum >> 16) & 0xFF;
    data[58] = (checksum >> 8) & 0xFF;
    data[59] = (checksum) & 0xFF;
    //SOAR_PRINT("checksum: %d\n", checksum);

    //Store beginning of packet
    data[60] = 0b10100101
    data[61] = 0b10100101;
    data[62] = 0b00000000;
    data[63] = 0b11111111;   

    //Write to relevant sector
    //byte address is not the same as bit address
    addressToWrite = si_currentInformation.offset + 8192;
    for(int i = 0; i < 64; i++) {
        W25qxx_WriteByte(data[i], addressToWrite + i);
    }

    //for debugging
    //uint8_t* sector1Data = new uint8_t[16];
    //uint8_t* sector2Data = new uint8_t[16];
    //W25qxx_ReadBytes(sector1Data, w25qxx.SectorSize * 0, 16);
    //W25qxx_ReadBytes(sector2Data, w25qxx.SectorSize * 1, 16);

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
        si_currentInformation = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
    if(cm.GetTaskCommand() == 1) 
    {
        rs_currentInformation.State = (RocketState) (cm.GetDataPointer())[0];
        WriteStateToFlash();
        SOAR_PRINT("state written to flash");
    }
    cm.Reset();
}