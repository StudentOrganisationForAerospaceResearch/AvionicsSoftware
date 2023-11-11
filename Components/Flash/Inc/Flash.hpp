/**
 * @file    Flash.hpp
 * @brief   This file contains the class declaration for a flash device driver.
 *
 */
#ifndef FLASH_BASE_HPP
#define FLASH_BASE_HPP
#include "SystemDefines.hpp"

class Flash {
   public:
    virtual void Init() = 0;
    virtual bool Erase(uint32_t offset) = 0;
    virtual bool Write(uint32_t offset, uint8_t* data, uint32_t len) = 0;
    virtual bool Read(uint32_t offset, uint8_t* data, uint32_t len) = 0;
    virtual bool EraseChip() = 0;

    virtual uint32_t GetSectorSize() = 0;
};

#endif  // FLASH_BASE_HPP
