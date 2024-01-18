/**
 ******************************************************************************
 * File Name          : SafeSimpleDualSectorStorage.hpp
 * Description        : SSDSS is an implementation of SimpleDualSectorStorage
 *                      with a mutex to allow for safe access from multiple
 *                      threads.
 *
 * Note               : This class is only safe if the underlying Flash driver
 *                      provided is thread safe.
 ******************************************************************************
*/
#ifndef SOAR_SAFE_SIMPLE_DUAL_SECTOR_STORAGE_HPP
#define SOAR_SAFE_SIMPLE_DUAL_SECTOR_STORAGE_HPP

#include "SimpleDualSectorStorage.hpp"

// Macros/Constexprs ---------------------------------------------------------------------
constexpr uint16_t SSDSS_READ_WRITE_MUTEX_TIMEOUT_MS = 1000;  // Waits up to 1 second for the mutex to be available
constexpr uint16_t SSDSS_MAINTAIN_MUTEX_TIMEOUT_MS = 0;       // Does not wait for the mutex to be available

// Class ---------------------------------------------------------------------
/**
 * @brief Safe Simple Dual Sector Storage Class
 *        is a SimpleDualSectorStorage with a mutex to allow for safe access
 *        from multiple threads.
 *
 * @note  This class is ONLY safe to be used if the Flash driver provided is
 *        thread safe
 *
 * @tparam T Type of data to store
 */
template <typename T>
class SafeSimpleDualSectorStorage : private SimpleDualSectorStorage<T> {
   public:
    SafeSimpleDualSectorStorage(Flash* flashDriver, uint32_t startAddr);

    bool Read(T& data);
    bool Write(T& data);

    void Maintain();

    void Erase();

   private:
    Mutex mutex_;
};

template <typename T>
SafeSimpleDualSectorStorage<T>::SafeSimpleDualSectorStorage(Flash* flashDriver, uint32_t startAddr)
    : SimpleDualSectorStorage<T>(flashDriver, startAddr) {}

/**
 * @brief Reads data from the Safe Simple Dual Sector Storage instance.
 *        Waits a while until the mutex can be acquired.
 *
 * @param data The data to read
 * @return true if successful, false otherwise
 */

template <typename T>
bool SafeSimpleDualSectorStorage<T>::Read(T& data) {
    bool success = false;
    if (mutex_.Lock(SSDSS_READ_WRITE_MUTEX_TIMEOUT_MS)) {
        success = SimpleDualSectorStorage<T>::Read(data);
        mutex_.Unlock();
    }
    return success;
}

/**
 * @brief Writes data to the Safe Simple Dual Sector Storage.
 *        Waits a while until the mutex can be acquired.
 *
 * @param data The data to write
 * @return true if successful, false otherwise
 */
template <typename T>
bool SafeSimpleDualSectorStorage<T>::Write(T& data) {
    bool success = false;
    if (mutex_.Lock(SSDSS_READ_WRITE_MUTEX_TIMEOUT_MS)) {
        success = SimpleDualSectorStorage<T>::Write(data);
        mutex_.Unlock();
    }
    return success;
}

/**
 * @brief Maintains the Safe Simple Dual Sector Storage.
 *        Only runs maintain if the mutex can be instantly acquired.
 */
template <typename T>
void SafeSimpleDualSectorStorage<T>::Maintain() {
    if (mutex_.Lock(SSDSS_MAINTAIN_MUTEX_TIMEOUT_MS)) {
        SimpleDualSectorStorage<T>::Maintain();
        mutex_.Unlock();
    }
}

/**
 * @brief Erases both sectors of the Safe Simple Dual Sector Storage.
 *        Waits forever for the mutex to be acquired.
 */
template <typename T>
void SafeSimpleDualSectorStorage<T>::Erase() {
    if (mutex_.Lock()) {
        SimpleDualSectorStorage<T>::Erase();
        mutex_.Unlock();
    }
}

#endif  // SOAR_SAFE_SIMPLE_DUAL_SECTOR_STORAGE_HPP
