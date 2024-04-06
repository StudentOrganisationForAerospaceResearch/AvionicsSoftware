/**
 ******************************************************************************
 * File Name          : MEVManager.hpp
 * Description        : Servo State and Control Actions
 ******************************************************************************
*/

#ifndef MEV_MANAGER_HPP
#define MEV_MANAGER_HPP

#include "DMBProtocolTask.hpp"
#include "PBBRxProtocolTask.hpp"

class MEVManager {
public:
    enum MEVState {
        INDETERMINATE,
        OPEN,
        CLOSE
    };

    static void MEV_OPEN();
    static void MEV_CLOSE();

private:
    static MEVState shouldMevBeOpen;
};

#endif // MEV_MANAGER_HPP
