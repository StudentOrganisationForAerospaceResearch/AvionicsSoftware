/**
 ******************************************************************************
 * File Name          : SimpleSectorStorage.hpp
 * Description        : Primitive sector storage class, holding data at the start
 *                      of a particular sector
 * Author             : cjchanx (Chris)
 ******************************************************************************
*/
#ifndef SOAR_SIMPLE_SECTOR_STORAGE_HPP_
#define SOAR_SIMPLE_SECTOR_STORAGE_HPP_
#include "SystemDefines.hpp"
#include "Flash.hpp"
#include "Utils.hpp"

// Macros/Constexprs ---------------------------------------------------------------------
constexpr uint8_t SSS_HEADER_BYTE = 0xE5; // Simple Sector Storage Header Byte

// General Functions ---------------------------------------------------------------------
static uint16_t SSS_CalculateChecksum(uint8_t* data, uint16_t len)
{
    //TODO: Change this with the Utils checksum function once that is merged
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

// Class ----------------------------------------------------------------------------------
/**
 * @brief Simple Sector Storage Class
 *        Holds data at the start of a particular sector, and wraps it in a header
 *        and a checksum
 *
 * @tparam T Type of data to store
 */
template<typename T>
class SimpleSectorStorage
{
public:
    SimpleSectorStorage(Flash* flashDriver, uint32_t startAddr);

    bool Write(T& data, bool checkErased = true);
    bool Read(T& data);

    bool Erase();
    bool Invalidate();

protected:
    // Data format in flash:
    struct Data
    {
        uint8_t header{ SSS_HEADER_BYTE };
        T data;
        uint16_t crc;
    };

    // Helper functions
    void AddCRC(Data& data);
    bool IsCRCValid(Data& data);

    // Variables
    T validData_;
    bool hasValidData_;

    // Constants
    const uint32_t kStartAddr_;
    Flash* kFlash_;
};

// Function Implementations ----------------------------------------------------------------------------------

/**
* @brief Constructs a new SimpleSectorStorage object
*
* @param flashDriver pointer to a Flash object used to interface with flash memory, assumed to be initialized
* @param startAddr starting address of the sector to store data in.
*
* @note If the startAddr is not on a sector boundary, it will be rounded down to the nearest valid sector boundary.
*
* @tparam T type of data to store
*/
template <typename T>
SimpleSectorStorage<T>::SimpleSectorStorage(Flash* flashDriver, uint32_t startAddr) :
    kStartAddr_(startAddr),
    kFlash_(flashDriver)
{
    validData_ = { 0 };
    hasValidData_ = false;

    // Print a warning if the start address is not on a sector boundary
    if (kStartAddr_ % kFlash_->GetSectorSize() != 0)
    {
        SOAR_PRINT("Warn.SSS: Start address not on sector boundary!\n");
    }
}


/**
* @brief Writes data of type T to flash memory. Erases the sector if necessary.
*
* @param data data to be written
* @param checkErased whether to check if the sector is erased before writing
*
* @return true if write was successful, false otherwise
*/
template <typename T>
bool SimpleSectorStorage<T>::Write(T& data, bool checkErased)
{
    if(checkErased) {
        // Read from the flash and verify that the storage is erased
        uint8_t readData[sizeof(Data)] = { 0 };
        bool successRead = kFlash_->Read(kStartAddr_, readData, sizeof(Data));
        if (!successRead)
            return false;

        // Verify the data is erased, if it is not, erase it
        for (uint16_t i = 0; i < sizeof(Data); i++)
        {
            if (readData[i] != 0xFF)
            {
                SOAR_PRINT("Warn.SSS: - Sector Not Erased on Write\n");
                bool successErase = kFlash_->Erase(kStartAddr_);
                if (!successErase)
                    return false;
                break;
            }
        }
    }

    // Make a new data object
    Data dataToWrite;

    // Setup the data to write
    dataToWrite.header = SSS_HEADER_BYTE;
    dataToWrite.data = data;
    AddCRC(dataToWrite);

    // Write the data to flash memory
    bool successWrite = kFlash_->Write(kStartAddr_, reinterpret_cast<uint8_t*>(&dataToWrite), sizeof(Data));
    if (!successWrite)
        return false;

    // Add the data to the cache
    validData_ = data;
    hasValidData_ = true;
    return true;
}

/**
* @brief Reads data of type T from flash memory, returns the stored data if it has already been read.
*
* @param data to be read into
*
* @return true if read found valid data at the start of the sector
*/
template <typename T>
bool SimpleSectorStorage<T>::Read(T& data)
{
    // Check if cached data is valid 
    if (hasValidData_)
    {
        data = validData_;
        return true;
    }

    Data readData;

    // Read from flash 
    bool successRead = kFlash_->Read(kStartAddr_, reinterpret_cast<uint8_t*>(&readData), sizeof(Data));

    // If the read was not successful, return false
    if (!successRead)
        return false;

    // If either the header or the CRC is invalid, return false
    if (readData.header != SSS_HEADER_BYTE || !IsCRCValid(readData))
        return false;

    // Data is valid, cache the data, update the reference and return true
    data = readData.data;
    validData_ = readData.data;
    hasValidData_ = true;
    
    return true;
}

/**
 * @brief Erase the sector in flash memory and invalidate cached data.
 *
 * @return true if sector was erased successfully, false otherwise.
 */
template <typename T>
bool SimpleSectorStorage<T>::Erase()
{
    // Erase the sector in flash 
    bool successErase = kFlash_->Erase(kStartAddr_);
    if (!successErase)
        return false;

    // Invalidate the cached data
    validData_ = { 0 };
    hasValidData_ = false;

    return true;
}

/**
 * @brief Invalidates the data in the sector by writing a 0x00 to the header byte.
 *        Invalidates cached data.
 *
 *        Invalidating is much faster than erasing,
 *        but it does not free up the sector for use.
 *
 * @return true if data was invalidated successfully, false otherwise.
 */
template <typename T>
bool SimpleSectorStorage<T>::Invalidate()
{
    // Write a 0x00 to the header byte
    uint8_t writeBuffer = 0x00;
    bool successWrite = kFlash_->Write(kStartAddr_, &writeBuffer ,sizeof(writeBuffer));

    // If we failed to write, return false
    if (!successWrite)
        return false;

    // Invalidate the cached data
    validData_ = { 0 };
    hasValidData_ = false;

    return true;
}


// Helper Function Implementations ----------------------------------------------------------------------------------

/**
* @brief Calculate the CRC for the Data struct, and store it in the data parameter.
*
* @param data to calculate the CRC for
*
* @return void
*/
template <typename T>
void SimpleSectorStorage<T>::AddCRC(Data& data)
{
    uint8_t* byteData = reinterpret_cast<uint8_t*>(&data);

    // Calculate CRC of the data, excluding the crc field
    uint16_t crc = SSS_CalculateChecksum(byteData + 1, sizeof(Data) - sizeof(uint16_t));

    data.crc = crc;
}

/**
* @brief Check if the CRC of the data struct is valid.
*
* @param data to check the CRC for
*
* @return true if CRC is valid, false otherwise
*/
template <typename T>
bool SimpleSectorStorage<T>::IsCRCValid(Data& data)
{
    uint8_t* byteData = reinterpret_cast<uint8_t*>(&data);

    uint16_t crc = SSS_CalculateChecksum(byteData, sizeof(Data) - sizeof(uint16_t));

    return (crc == data.crc);
}


#endif // SOAR_SIMPLE_SECTOR_STORAGE_HPP_
