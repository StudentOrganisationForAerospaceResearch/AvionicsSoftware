/**
 ******************************************************************************
 * @file   SPIFlash.hpp
 * @brief  Wrapper for SPI flash memory operations
 ******************************************************************************
 */
#ifndef SPIFLASH_WRAPPER_HPP_
#define SPIFLASH_WRAPPER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "w25qxx.hpp"
#include "w25qxxConf.hpp"
#include "SystemDefines.hpp"
#include "Flash.hpp"

/* SPI Flash ------------------------------------------------------------------*/
/**
* @brief Wrapper for SPI flash memory operations
*/
class SPIFlash : public Flash
{
public:
    /**
     * @brief Initialize the W25qxx SPI Flash Memory device
     */
    void Init() override
    {
        W25qxx_Init();
    }

    /**
    * @brief Erase the sector containing the given offset.
    *
    * This method erases the sector from the flash memory that encompasses
    * the specified offset.
    *
    * @param offset The offset where the sector to be erased is located.
    * @return Returns 'true' if the sector was erased successfully or 'false'
    *         if the offset is outside the flash memory's valid boundaries.
    */
    bool Erase(uint32_t offset) override
    {
        if (offset >= (w25qxx.SectorSize * w25qxx.SectorCount))
            return false;

        uint32_t SectorAddr = (offset / w25qxx.SectorSize);
        W25qxx_EraseSector(SectorAddr);
        return true;
    }


    /**
    * @brief Write data to flash memory at specified offset
    * @param offset offset at which to write data
    * @param data pointer to data to be written
    * @param len length of data to be written up to the sector size (4096 bytes)
    * @return Returns 'true' if the write operation was successful, or 'false' if the offset
    *         is outside the flash memory's valid boundaries or the length of data to be written
    *         is greater than the sector size.
    */
    bool Write(uint32_t offset, uint8_t* data, uint32_t len) override
    {
        if (offset >= (w25qxx.SectorSize * w25qxx.SectorCount) || len > w25qxx.SectorSize)
            return false;

        uint32_t SectorAddr = (offset / w25qxx.SectorSize);
        uint32_t OffsetInSector = offset % w25qxx.SectorSize;
        W25qxx_WriteSector(data, SectorAddr, OffsetInSector, len);
        return true;
    }

    /**
     * @brief Read data from flash memory at specified offset
     * @param offset offset at which to read data
     * @param data pointer to buffer where data should be stored
     * @param len length of data to be read up to the sector size (4096 bytes)
     * @return Returns 'true' if the read operation was successful, or 'false' if the offset
     *         is outside the flash memory's valid boundaries, or the len is greater
     *         than the sector size.
     */
    bool Read(uint32_t offset, uint8_t* data, uint32_t len) override
    {
        if (offset >= (w25qxx.SectorSize * w25qxx.SectorCount) || len > w25qxx.SectorSize)
            return false;

        uint32_t SectorAddr = (offset / w25qxx.SectorSize);
        uint32_t OffsetInSector = offset % w25qxx.SectorSize;
        W25qxx_ReadSector(data, SectorAddr, OffsetInSector, len);
        return true;
    }

    /**
     * @brief Erase the entire W25qxx SPI Flash Memory device.
     *
     * This method erases the entire flash memory, effectively resetting it to an initial state.
     *
     * @param None
     * @return Returns 'true' if the device was erased successfully and 'false'
     *         otherwise.
     */
    bool EraseChip() override
    {
        W25qxx_EraseChip();
        return true;
    }

    /**
     * @brief Get the sector size of the W25qxx SPI Flash Memory device.
     *
     * This method returns the sector size of the W25qxx SPI Flash Memory device, which is a property of the underlying flash memory.
     *
     * @return Returns the sector size of the W25qxx SPI Flash Memory device.
     */
    uint32_t GetSectorSize() override
    {
        return w25qxx.SectorSize;
    }
};

#endif // SPIFLASH_WRAPPER_HPP_