/**
 ******************************************************************************
 * File Name          : DualSectorStorage.hpp
 * Description        : Storage that spans across 2 sectors
 ******************************************************************************
*/
#ifndef SOAR_DUAL_SECTOR_STORAGE_HPP_
#define SOAR_DUAL_SECTOR_STORAGE_HPP_
#include "SystemDefines.hpp"
#include "w25qxx.hpp"

// Macros/Constexprs ---------------------------------------------------------------------
constexpr uint8_t DSS_HEADER_BYTE = 0xD5; // Dual Sector Storage Header Byte
constexpr bool DEFAULT_VERIFY_CHECKSUM_ON_SEEK = true; // Verify checksum on seek by default

uint16_t CalcDSSCRC(uint8_t* data, uint16_t size)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < size; i++)
    {
        crc += data[i];
    }
    return crc;
}

template <typename T>
class DualSectorStorage
{
public:
    DualSectorStorage(uint32_t startOffset, uint16_t sectorSize)
    {
        kStartOffset_ = 0;
        kSectorSize_ = 0;
        kPageSize_ = 0;

        bool bHasSeeked_ = false;

        currentWritingFlashPage_ = 0;
        currentWritingFlashIndex_ = 0;
        currentWritingFlashSector1_ = false;

    }
    ~DualSectorStorage();

    void Init();


protected:

    /**
     * @brief Data storage format inside the flash
     */
    struct Data
    {
        uint8_t header { DSS_HEADER_BYTE };
        uint16_t seqN; // Note: we need a special case for seqN rollover, we must store to the next sector with seqN = 0 then erase the current sector immedietly after
        T data;
        uint16_t CRC;
    };

    /**
     * @brief Seek function. Looks for the most recent data in the flash
     * @return true if seek succeded, false otherwise. In case of false, both sectors have been erased
     */
    inline bool Seek()
    {
        uint16_t pageNSec1 = 0;
        int16_t emptyIndexSec1 = -1;

        // Seek in sector 0
        Data* sector0MostRecentData = SeekSector(false, pageNSec1, emptyIndexSec1);

        uint16_t pageNSec2 = 0;
        uint16_t emptyIndexSec2 = -1;

        // Seek in sector 1
        Data* sector1MostRecentData = SeekSector(true, pageNSec2, emptyIndexSec2);

        bHasSeeked_ = true;

        uint8_t takeSectorX = 2; // Which sector to take data from, 0 = sector 0, 1 = sector 1, 2 = none

        if (sector0MostRecentData == nullptr && sector1MostRecentData == nullptr) {
            takeSectorX = 2;
        }
        else if(sector1MostRecentData == nullptr) {
            // Sector 0 data valid
            takeSectorX = 0;
        }
        else if(sector0MostRecentData == nullptr) {
            // Sector 1 data valid
            takeSectorX = 1;
        }
        else if(sector0MostRecentData && sector1MostRecentData) {
            // Both sectors valid
            if(sector0MostRecentData->seqN > sector1MostRecentData->seqN) {
                // Sector 0 data is more recent, erase sector 1
                takeSectorX = 0;
                Erase(true);
            }
            else {
                // Sector 1 data is more recent or equal, erase sector 0
                takeSectorX = 1;
                Erase(false);
            }
        }
        else {
            // Should never get here
            return false;
        }

        // Update the sector info
        if(takeSectorX == 0) {
            latestData_ = sector0MostRecentData->data;
            currentWritingFlashIndex_ = emptyIndexSec0;
            currentWritingFlashPage_ = pageNSec0;
            currentWritingFlashSector1_ = false;
        }
        else if(takeSectorX == 1) {
            latestData_ = sector1MostRecentData->data;
            currentWritingFlashIndex_ = emptyIndexSec1;
            currentWritingFlashPage_ = pageNSec1;
            currentWritingFlashSector1_ = true;
        }
        else {
            Erase(false);
            Erase(true);
            currentWritingFlashIndex_ = 0;
            currentWritingFlashPage_ = 0;
            currentWritingFlashSector1_ = 0;
            return false;
        }

        return true;
    }

    /**
     * @brief Seeks the most recent data in the current sector
     * @param bSector1 true if we are seeking in sector 1, false if we are seeking in sector 0
     * @param rPageN the page number of the most recent data (ignore if not found)
     * @param rEmptyIndex returns the index of the empty data if found, -1 otherwise
     * @return pointer to the most recent data, nullptr if no data found
     */
    Data* SeekSector(bool bSector1, uint16_t& rPageN, int16_t& rEmptyIndex)
    {
        uint16_t pageN = 0;
        int16_t emptyIndex = -1;
        Data* mostRecentData = nullptr;

        // Start with sector 0, and attempt to find the most recent data until we hit an empty page
        do {
            Read(false, pageN++);

            emptyIndex = SeekEmptyDataReadIndex();
            Data* currRecentData = SeekMostRecentData();

            // Update to one with newest seqN
            if (mostRecentData && currRecentData)
            {
                if (currRecentData->seqN > mostRecentData->seqN)
                {
                    mostRecentData = currRecentData;
                }
            }
            else if (currRecentData)
            {
                mostRecentData = currRecentData;
            }

        } while (emptyIndex == -1 && pageN < GetPagesPerSector());

        // Set the return values
        rPageN = pageN;
        rEmptyIndex = emptyIndex;

        return mostRecentData;
    }

    /**
     * @brief Write function
     * @param flashIndex - offset from the start of the DualSectorStorage in multiples of sizeof(Data)
     * @param data - data to write
     */
    inline bool Write(T& src)
    {
        // First check for sequence number overflow, this requires us to go to the next sector
        if (latestData_.seqN == UINT16_MAX) {
            // Make sure the opposite sector is empty
            Read(!currentWritingFlashSector1_, 0);
            if (SeekEmptyDataReadIndex() != 0) {
                SOAR_PRINT("DSS - Opposite Sector Invalid on Write - Erasing...\n");
                Erase(!currentWritingFlashSector1_);
            }

            // Write current data to the next sector
            //TODO: This is so complicated... check this
            W25qxx_WriteSector((uint8_t*)&latestData_, kStartOffset_ / kSectorSize_ + (!currentWritingFlashSector1_ ? 1 : 0), 0, sizeof(Data));

            // Erase the current sector
            Erase(currentWritingFlashSector1_);

            // Reset the sequence number
            latestData_.seqN = 0;

            // Now, we have to zero out the sequence number of the data that was just written (this should work, since we're setting it to 0)
            W25qxx_WriteSector((uint8_t*)&latestData_, kStartOffset_ / kSectorSize_ + (!currentWritingFlashSector1_ ? 1 : 0), 0, sizeof(Data));

            // We are now safely in the next sector, let's update the addressing
            currentWritingFlashSector1_ = !currentWritingFlashSector1_;
            currentWritingFlashIndex_ = 1;
            currentWritingFlashPage_ = 0;

            //TODO: We should probably do a write verify... ugh.. .this addressing system is so complicated... we need a function that just says 'IncreasePage()' etc.
        }

        // Package T into a Data struct
        Data data;
        data.header = DSS_HEADER_BYTE;
        data.seqN = latestData_.seqN + 1;
        data.data = src;
        AddDataChecksum(data);
        
        if (currentWritingFlashPage_ >= GetPagesPerSector()) {
            // Make sure the opposite sector is empty
            Read(!currentWritingFlashSector1_, 0);
            if (SeekEmptyDataReadIndex() != 0) {
                SOAR_PRINT("DSS - Opposite Sector Invalid on Write - Erasing...\n");
                Erase(!currentWritingFlashSector1_);
            }

            // Write new data to the next sector
            W25qxx_WriteSector((uint8_t*)&data, kStartOffset_ / kSectorSize_ + (!currentWritingFlashSector1_ ? 1 : 0), 0, sizeof(Data));

            // Write the data to the new sector
            //TODO: This needs to be done!
        }
        else if (currentWritingFlashIndex_ >= GetNumDataPerPage()) {
            // Past the end of the page, start using the other sector, then erase (some fatal error occured)

        }
        else {
            // Write the data to the current sector
            //TODO: Check addressing!
            W25qxx_WriteSector((uint8_t*)&data, kStartOffset_ / kSectorSize_ + (currentWritingFlashSector1_ ? 1 : 0), 
                currentWritingFlashPage_ * kPageSize_ + currentWritingFlashIndex_ * sizeof(Data), sizeof(Data));
        }
    }

    /**
     * @brief Read function. Reads a full page of data into the readArray
     * @param pageIndex - index of the page, inside the sector, to read
     * @param bSector1 - false if sector 0, true if sector 1
     */
    inline bool Read(bool bSector1, uint16_t pageIndex)
    {
        uint16_t readLen = GetNumValidBytesPerPage();

        // If the page index is out of bounds, return false
        if (pageIndex > GetLastPageIndex())
        {
            // Zero out the read array headers, since we use the header to determine if the data is valid
            for (uint16_t i = 0; i < readLen; i++)
            {
                readArray[i].header = 0; 
            }

            SOAR_PRINT("Error: DSS - Page index out of bounds\n");
            return false;
        }

        // Read the data
        W25Qxx_ReadBytes((uint8_t*)readArray, 
            kStartOffset_ + (bSector1 ? kSectorSize_ : 0) + (pageIndex * kPageSize_),
            readLen);

        return true;
    }

    //TODO: Since erase takes a while... we may want to make this non-blocking so state storage can return immedietly
    /**
     * @brief Erase function
     * @param bSector1 - false to erase sector 0, true to erase sector 1 (relative to the dual sector storage)
     */
    bool Erase(bool bSector1)
    {
        W25qxx_EraseSector(kStartOffset_ + (bSector1 ? kSectorSize_ : 0));
        return true;
    }

    /**
     * @brief Search within the current readData for an empty Data segment
     * @return -1 if no empty seg was found, otherwise the index of the empty seg
     */
    int16_t SeekEmptyDataReadIndex()
    {
        for (uint16_t dataN = 0; dataN < GetNumDataPerPage(); dataN++)
        {
            Data* data = &readArray[dataN];

            // For a fast-check, check the header
            if (data->header == 0xFF)
            {
                // Check the rest of the data is 0xFF
                bool bEmpty = true;
                
                // Check all remaining bytes of data is 0xFF
                for (uint16_t byteN = 1; byteN < sizeof(Data); byteN++)
                {
                    if (((uint8_t*)data)[byteN] != 0xFF)
                    {
                        bEmpty = false;
                        break;
                    }
                }

                // If the data is empty, return the index
                if (bEmpty)
                    return dataN;
            }
        }

        // If no empty data was found, return -1
        return -1;
    }

    /**
     * @brief Search within the current readData for the most recent data
     * @param bCheckMe - if true, the data will be checked for checksum before returning
     * @return nullptr if no data was found, otherwise a pointer to the most recent data
     */
    Data* SeekMostRecentData(bool bCheckMe = DEFAULT_VERIFY_CHECKSUM_ON_SEEK)
    {
        uint16_t latestSeqN = 0;
        Data* latestData = nullptr;
        for(uint16_t dataN = 0; dataN < GetNumDataPerPage(); dataN++) 
        {
            Data* data = &readArray[dataN];

            // If the header is valid, check the sequence number
            if (data->header == DSS_HEADER_BYTE)
            {
                // If the checksum is requested, check it
                if(bCheckMe)
                {
                    // Check the checksum
                    if (!CheckDataChecksum(data))
                        continue;
                }

                if (data->seqN >= latestSeqN)
                {
                    latestSeqN = data->seqN;
                    latestData = data;
                }
            }
        }

        return latestData;
    }

    /**
     * @brief Checks the data checksum to see if it is valid
     * @param data - pointer to the data to check
     * @return true if the checksum is valid, false otherwise
     */
    bool CheckDataChecksum(Data* data)
    {
        // Calculate the checksum
        uint16_t crc = CalcDataChecksum(data);

        // Compare the checksums
        if (crc == data->CRC)
            return true;
        else
            return false;
    }

    /**
     * @brief Calculates the data checksum
     * @param data - pointer to the data to calculate the checksum for
     * @return the checksum
     */
    uint16_t CalcDataChecksum(Data* data)
    {
        return CalcDSSCRC((uint8_t*)&data, GetChecksumedLength());
    }

    /**
     * @brief Adds checksum to the data
     * @param data - pointer to the data to add checksum to
     */
    void AddDataChecksum(Data* data)
    {
        data->CRC = CalcDataChecksum(data);
    }


    // Helpers --------------------------------------------------------------------------------
    const uint16_t GetLastPageIndex() { return (kSectorSize_ / kPageSize_) - 1; }
    const uint16_t GetPagesPerSector() { return kSectorSize_ / kPageSize_; }
    const uint16_t GetNumDataPerPage() { return kPageSize_ / sizeof(Data); }
    const uint16_t GetNumValidBytesPerPage() { return GetNumDataPerPage() * sizeof(Data); }

    const uint16_t GetChecksumedLength() { return sizeof(Data) - sizeof(uint16_t); }

protected:

    const uint32_t kStartOffset_;
    const uint16_t kSectorSize_;
    const uint16_t kPageSize_;

    bool bHasSeeked_;

    uint16_t currentWritingFlashIndex_;
    uint16_t currentWritingFlashPage_;
    bool currentWritingFlashSector1_;

    Data readArray[GetNumDataPerPage()];

    T latestData_;
};


#endif   // SOAR_DUAL_SECTOR_STORAGE_HPP_