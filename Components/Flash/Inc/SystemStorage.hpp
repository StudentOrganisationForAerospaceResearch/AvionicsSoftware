/**
 ******************************************************************************
 * File Name          : SystemStorage.hpp
 *                      A system wide singleton containing the system state
 *                      stored with a SafeDualSectorStorage.
 *
 *                      The FlightTask should have priority in doing
 *                      the first ever read. After which all other operations
 *                      (erase, write) are handled by FlashTask except for
 *                      time sensitive write operations.
 ******************************************************************************
*/
#ifndef SOAR_SYSTEM_STATE_STORAGE_HPP_
#define SOAR_SYSTEM_STATE_STORAGE_HPP
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "SafeSimpleDualSectorStorage.hpp"
#include "SPIFlash.hpp"

// Macros/Constexprs ---------------------------------------------------------------------
constexpr uint32_t SYSTEM_STORAGE_START_SECTOR_ADDR = SPI_FLASH_SYSTEM_SDSS_STORAGE_START_ADDR;


// System Info Struct ---------------------------------------------------------------------
struct SystemState
{
    RocketState rocketState;
};

// System Info Struct ---------------------------------------------------------------------
class SystemStorage : public SafeSimpleDualSectorStorage<SystemState>
{
public:
    // Singleton instance for SystemStorage
    static SystemStorage& Inst() {
        static SystemStorage inst;
        return inst;
    }

private:
    SystemStorage();
    SystemStorage(const SystemStorage&);                      // Prevent copy-construction
    SystemStorage& operator=(const SystemStorage&);           // Prevent assignment
};

inline SystemStorage::SystemStorage() :
    SafeSimpleDualSectorStorage<SystemState>(&SPIFlash::Inst(),
        SYSTEM_STORAGE_START_SECTOR_ADDR)
{
}

#endif // SOAR_SYSTEM_STATE_HPP
