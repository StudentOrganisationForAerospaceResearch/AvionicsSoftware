/**
 ******************************************************************************
 * @file   MockFlash.hpp
 * @brief  Mock flash memory operations
 ******************************************************************************
 */
#ifndef FLASH_MOCK_DRIVER_HPP_
#define FLASH_MOCK_DRIVER_HPP_

 /* Includes ------------------------------------------------------------------*/
#include "w25qxx.hpp"
#include "w25qxxConf.hpp"
#include "SystemDefines.hpp"
#include "Flash.hpp"

/* SPI Flash ------------------------------------------------------------------*/
/**
* @brief Wrapper for SPI flash memory operations
*/
class MockFlash : public Flash
{
public:
    /*
     * @brief Singleton instance
     */
    static MockFlash& Inst() {
        static MockFlash inst;
        return inst;
    }

    /**
     * @brief Initialize the Mock Flash Memory device
     */
    void Init() override
    {

    }

    /**
    * @brief Mock for Erase the sector containing the given offset
    */
    bool Erase(uint32_t offset) override
    {
		return true
    }


    /**
    * @brief Mock for Write data to flash memory at specified offset
    */
    bool Write(uint32_t offset, uint8_t* data, uint32_t len) override
    {
        return true;
    }

    /**
     * @brief Mock for Read data from flash memory at specified offset
     */
    bool Read(uint32_t offset, uint8_t* data, uint32_t len) override
    {
        return true;
    }

    /**
     * @brief Mock for Erase chip
     */
    bool EraseChip() override
    {
        return true;
    }

    /**
     * @brief Mock for sector size getter
     * @return Always returns 4096
     */
    uint32_t GetSectorSize() override
    {
        return 4096;
    }

    /**
	 * @brief Mock for IsInitialized, returns tue
     */
    bool GetInitialized()
    {
        return true;
    }

private:
    // Private Functions
    MockFlash() { isInitialized_ = false; }          // Private constructor
    MockFlash(const MockFlash&);                      // Prevent copy-construction
    MockFlash& operator=(const MockFlash&);           // Prevent assignment

    bool isInitialized_ = false;
};

#endif // FLASH_MOCK_DRIVER_HPP_
