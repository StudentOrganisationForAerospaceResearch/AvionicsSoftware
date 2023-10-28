/**
 ******************************************************************************
 * @file   SPIFlash.hpp
 * @brief  Wrapper for SPI flash memory operations
 ******************************************************************************
 */
#ifndef SPIFLASH_WRAPPER_HPP_
#define SPIFLASH_WRAPPER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "Flash.hpp"
#include "SystemDefines.hpp"
#include "w25qxx.hpp"
#include "w25qxxConf.hpp"

/* Constants and Macros -------------------------------------------------------*/
constexpr uint16_t DEFAULT_FLASH_SECTOR_SIZE =
    4096;  // 4KB - Default If Flash was not Initialized (to prevent % by 0 errors)

/* SPI Flash ------------------------------------------------------------------*/
/**
* @brief Wrapper for SPI flash memory operations
*/
class SPIFlash : public Flash {
   public:
    /*
     * @brief Singleton instance
     */
    static SPIFlash& Inst() {
        static SPIFlash inst;
        return inst;
    }

    /**
     * @brief Initialize the W25qxx SPI Flash Memory device
     */
    void Init() override {
        if (!isInitialized_) {
            W25qxx_Init();
            isInitialized_ = true;
        }
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
    bool Erase(uint32_t offset) override {
        if (offset >= (w25qxx.SectorSize * w25qxx.SectorCount))
            return false;

        uint32_t SectorAddr = (offset / GetSectorSize());
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
    bool Write(uint32_t offset, uint8_t* data, uint32_t len) override {
        if (offset + len >= (w25qxx.SectorSize * w25qxx.SectorCount) ||
            len > w25qxx.SectorSize)
            return false;

        uint32_t SectorAddr = (offset / GetSectorSize());
        uint32_t OffsetInSector = offset % GetSectorSize();
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
    bool Read(uint32_t offset, uint8_t* data, uint32_t len) override {
        if (offset + len >= (w25qxx.SectorSize * w25qxx.SectorCount) ||
            len > w25qxx.SectorSize)
            return false;

        uint32_t SectorAddr = (offset / GetSectorSize());
        uint32_t OffsetInSector = offset % GetSectorSize();
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
    bool EraseChip() override {
        W25qxx_EraseChip();
        return true;
    }

    /**
     * @brief Get the sector size of the W25qxx SPI Flash Memory device.
     *
     * This method returns the sector size of the W25qxx SPI Flash Memory device, which is a property of the underlying flash memory.
     *
	 * @return Returns the sector size of the W25qxx SPI Flash Memory device. Returns 4096 if the device is not initialized.
     */
    uint32_t GetSectorSize() override {
        if (w25qxx.SectorSize == 0)
            return DEFAULT_FLASH_SECTOR_SIZE;

        return w25qxx.SectorSize;
    }

    /**
     * @brief Gets if the flash is initialized or not
     */
    bool GetInitialized() {
        return (isInitialized_ && (w25qxx.Lock == 0) && (w25qxx.ID == W25Q512));
    }

   private:
    // Private Functions
    SPIFlash() { isInitialized_ = false; }  // Private constructor
    SPIFlash(const SPIFlash&);              // Prevent copy-construction
    SPIFlash& operator=(const SPIFlash&);   // Prevent assignment

    bool isInitialized_ = false;
};

#endif  // SPIFLASH_WRAPPER_HPP_
