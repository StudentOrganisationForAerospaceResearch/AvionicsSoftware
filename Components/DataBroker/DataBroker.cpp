/**
 ********************************************************************************
 * @file    DataBroker.cpp
 * @author  shivam
 * @date    Nov 23, 2024
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "DataBroker.hpp"
#include "Publisher.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/
// clang-format off
//DataBroker::Publisher<IMUData> DataBroker::IMU_Data_publisher(DataBrokerMessageTypes::IMU_DATA);
//DataBroker::Publisher<ThermocoupleData> DataBroker::Thermocouple_Data_publisher(DataBrokerMessageTypes::THERMOCOUPLE_DATA);
Publisher<IMUData> DataBroker::IMU_Data_publisher{DataBrokerMessageTypes::IMU_DATA};
Publisher<ThermocoupleData> DataBroker::Thermocouple_Data_publisher{DataBrokerMessageTypes::THERMOCOUPLE_DATA};

// clang-format on
/************************************
 * FUNCTION DECLARATIONS
 ************************************/

/************************************
 * FUNCTION DEFINITIONS
 ************************************/
