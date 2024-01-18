/**
 ******************************************************************************
 * File Name          : SimpleDualSectorStorage.hpp
 * Description        : Dual sector storage, storing data only at the start of
 *                      the sector, based on the SimpleSectorStorage class
 * Author             : cjchanx (Chris), MarceloLiGonzales (Marcelo)
 ******************************************************************************
*/
#ifndef SOAR_SIMPLE_DUAL_SECTOR_STORAGE_HPP
#define SOAR_SIMPLE_DUAL_SECTOR_STORAGE_HPP
#include "SimpleSectorStorage.hpp"

// Macros/Constexprs ---------------------------------------------------------------------

// Class ----------------------------------------------------------------------------------
/**
 * @brief Simple Dual Sector Storage Class
 *        Holds data at the start of a particular sector in two sectors,
 *        with a sequence number. Erases the older sector when writing.
 *        At any point in time, there should be a recoverable copy of the data
 *
 * @note  This class does not do wear leveling, and is not meant to be used
 *        for data that is written to frequently
 *
 * @tparam T Type of data to store
 */
template <typename T>
class SimpleDualSectorStorage {
   public:
    SimpleDualSectorStorage(Flash* flashDriver, uint32_t startAddr);

    bool Read(T& data);
    bool Write(T& data);

    void Maintain();

    void Erase();

   protected:
    struct Data {
        uint16_t seqN;
        T data;
    };

    enum CurrentValidSector {
        SECTOR_1,  // Sector 1 is valid
        SECTOR_2,  // Sector 2 is valid

        UNKNOWN,  // Initial state
        INVALID   // Both sectors are invalid
    };

    enum PendingOperation {
        NONE,  // No pending operations

        ERASE_SECTOR_1,  // Erase sector 1
        ERASE_SECTOR_2,  // Erase sector 2
    };

    CurrentValidSector validSector_;
    PendingOperation pendingOp_;

    SimpleSectorStorage<Data> sector1_;
    SimpleSectorStorage<Data> sector2_;
};

// Function Implementations -------------------------------------------------------------------------
/**
 * @brief Constructor for SimpleDualSectorStorage Class
 *
 * @param flashDriver instance of Flash class
 * @param startAddr start address of the sector to hold the data
 *
 * @tparam T type of data to store
 */
template <typename T>
SimpleDualSectorStorage<T>::SimpleDualSectorStorage(Flash* flashDriver, uint32_t startAddr)
    : sector1_(flashDriver, startAddr), sector2_(flashDriver, startAddr + flashDriver->GetSectorSize()) {
    // The startAddr must align with a sector boundary
    SOAR_ASSERT(startAddr % flashDriver->GetSectorSize() == 0, "Error-SDSS: Invalid startAddr");

    // Init variables
    validSector_ = UNKNOWN;
}

/**
 * @brief Read data from the valid sector, if unknown, finds the valid sector
 *        and updates the valid sector variable. Does not perform maintenance
 *        to read as fast as possible. Maintain() must be run later to ensure
 *        the state of the SDSS is correct.
 *
 * @param[out] data reference to data to be read
 * @return bool true if read was successful, false otherwise
 *
 * @tparam T type of data to read
 */
template <typename T>
bool SimpleDualSectorStorage<T>::Read(T& data) {
    bool readSuccess = true;

    // If we don't know the valid sector, figure it out. Otherwise use the valid sector data.
    if (validSector_ == UNKNOWN) {
        Data s1Data, s2Data;

        bool s1Valid = sector1_.Read(s1Data);
        bool s2Valid = sector2_.Read(s2Data);

        if (s1Valid && !s2Valid) {
            data = s1Data.data;
            validSector_ = SECTOR_1;
        } else if (!s1Valid && s2Valid) {
            data = s2Data.data;
            validSector_ = SECTOR_2;
        } else if (s1Valid && s2Valid) {
            if (s1Data.seqN > s2Data.seqN) {
                data = s1Data.data;
                validSector_ = SECTOR_1;
                pendingOp_ = ERASE_SECTOR_2;
            } else if (s1Data.seqN < s2Data.seqN) {
                data = s2Data.data;
                validSector_ = SECTOR_2;
                pendingOp_ = ERASE_SECTOR_1;
            } else {
                // Same seqN on both, take the one which appears first in the flash memory (ie, sector 1)
                data = s1Data.data;
                validSector_ = SECTOR_1;
                pendingOp_ = ERASE_SECTOR_2;
            }
        } else {
            validSector_ = INVALID;
            readSuccess = false;
        }
    } else if (validSector_ == SECTOR_1) {
        Data readData;
        readSuccess = sector1_.Read(readData);
        data = readData.data;
    } else if (validSector_ == SECTOR_2) {
        Data readData;
        readSuccess = sector2_.Read(readData);
        data = readData.data;
    } else {
        readSuccess = false;
    }

    return readSuccess;
}

/**
 * @brief Maintenance function for SimpleDualSectorStorage Class
 *        Runs any pending operations
 *
 * @tparam T type of data to store
 */
template <typename T>
void SimpleDualSectorStorage<T>::Maintain() {
    // If no pending operation, do nothing
    if (pendingOp_ == NONE)
        return;

    // Erase sector operations
    if (pendingOp_ == ERASE_SECTOR_1) {
        sector1_.Erase();
    } else if (pendingOp_ == ERASE_SECTOR_2) {
        sector2_.Erase();
    }

    // Clear the pending operation
    pendingOp_ = NONE;
}

/**
 * @brief Write data to the valid sector, if unknown, finds the valid sector
 *        and updates the valid sector variable. Does not perform maintenance
 *        to write as fast as possible. Maintain() must be run later to ensure
 *        the state of the SDSS is correct.
 *
 * @param[in] data reference to data to be written
 * @return bool true if write was successful, false otherwise
 *
 * @tparam T type of data to write
 */
template <typename T>
bool SimpleDualSectorStorage<T>::Write(T& data) {
    // First read the current valid data
    Data currentData;
    bool readSuccess = Read(currentData.data);

    // Run any pending operations
    Maintain();

    // If there is currently no valid data, write to sector 1
    if (!readSuccess) {
        currentData.seqN = 0;
        currentData.data = data;
        validSector_ = SECTOR_1;
        return sector1_.Write(currentData);
    }

    // If the sequence number is wrapping around, treat the current seqN as 0
    if (currentData.seqN >= 0xFFFF)
        currentData.seqN = 0;

    // Reusing the currentData variable, we wrap the data to be written in the Data struct
    currentData.seqN++;
    currentData.data = data;

    bool writeSuccess = true;

    // Write to the other sector
    if (validSector_ == SECTOR_1) {
        // Write to sector 2
        writeSuccess = sector2_.Write(currentData, true);

        // Invalidate sector 1 and set pending operation
        sector1_.Invalidate();
        validSector_ = SECTOR_2;
        pendingOp_ = ERASE_SECTOR_1;
    } else if (validSector_ == SECTOR_2) {
        // Write to sector 1
        writeSuccess = sector1_.Write(currentData, true);

        // Invalidate sector 2 and set pending operation
        sector2_.Invalidate();
        validSector_ = SECTOR_1;
        pendingOp_ = ERASE_SECTOR_2;
    } else {
        writeSuccess = false;
    }

    return writeSuccess;
}

/**
 * @brief Erases both sectors, should only be used on system startup or when
 *        completely resetting the storage. Blocks until both sectors are erased.
 */
template <typename T>
void SimpleDualSectorStorage<T>::Erase() {
    sector1_.Erase();
    sector2_.Erase();
    validSector_ = INVALID;
    pendingOp_ = NONE;
}

#endif  // SOAR_SIMPLE_DUAL_SECTOR_STORAGE_HPP
