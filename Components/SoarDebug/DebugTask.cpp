/**
  ******************************************************************************
  * File Name          : Debug.cpp
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "DebugTask.hpp"
#include "Command.hpp"
#include "Utils.hpp"
#include <cstring>

#include "FlightTask.hpp"
#include "GPIO.hpp"
#include "stm32f4xx_hal.h"

// External Tasks (to send debug commands to)
#include "BarometerTask.hpp"
#include "IMUTask.hpp"
#include "DMBProtocolTask.hpp"
#include "PBBRxProtocolTask.hpp"
#include "WatchdogTask.hpp"
#include "TimerTransitions.hpp"
#include "PressureTransducerTask.hpp"
#include "BatteryTask.hpp"
#include "GPSTask.hpp"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* HAL Callbacks ----------------------------------------------------------------*/
/**
 * @brief HAL Callback for DMA/Interrupt Complete
 *
 * TODO: This should eventually be in DMAController/main_avionics/UARTTask depending on how many tasks use DMA vs Interrupt vs Polling
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == SystemHandles::UART_Debug->Instance)
        DebugTask::Inst().InterruptRxData();
    else if (huart->Instance == SystemHandles::UART_Protocol->Instance)
        DMBProtocolTask::Inst().InterruptRxData();
    else if (huart->Instance == SystemHandles::UART_PBB->Instance)
        PBBRxProtocolTask::Inst().InterruptRxData();
    else if (huart->Instance == SystemHandles::UART_GPS->Instance)
        GPSTask::Inst().HandleGPSRxComplete();
}

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Constructor, sets all member variables
 */
DebugTask::DebugTask() : Task(TASK_DEBUG_QUEUE_DEPTH_OBJS)
{
    memset(debugBuffer, 0, sizeof(debugBuffer));
    debugMsgIdx = 0;
    isDebugMsgReady = false;
}

/**
 * @brief Init task for RTOS
 */
void DebugTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Debug task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)DebugTask::RunTask,
            (const char*)"DebugTask",
            (uint16_t)TASK_DEBUG_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_DEBUG_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "DebugTask::InitTask - xTaskCreate() failed");
}

// TODO: Only run thread when appropriate GPIO pin pulled HIGH (or by define)
/**
 *    @brief Runcode for the DebugTask
 */
void DebugTask::Run(void * pvParams)
{
    // Arm the interrupt
    ReceiveData();

    while (1) {
        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

        //Process the command
        if(cm.GetCommand() == DATA_COMMAND && cm.GetTaskCommand() == EVENT_DEBUG_RX_COMPLETE) {
            HandleDebugMessage((const char*)debugBuffer);
        }

        cm.Reset();
    }
}

/**
 * @brief Handles debug messages, assumes msg is null terminated
 * @param msg Message to read, must be null termianted
 */
void DebugTask::HandleDebugMessage(const char* msg)
{
    //-- PARAMETRIZED COMMANDS -- (Must be first)
    if (strncmp(msg, "rsc ", 4) == 0) {
        // Get parameter and send as a control action to flight task
        int32_t state = ExtractIntParameter(msg, 4);
        if (state != ERRVAL && state > 0 && state < UINT16_MAX)
            FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, state));
    }
    else if (strncmp(msg, "setradiohb ", 11) == 0) {
        // Send the heartbeat set to the watchdog task, where val is seconds
        int32_t val = ExtractIntParameter(msg, 11);
        if (val != ERRVAL)
            WatchdogTask::Inst().SendCommand(Command(RADIOHB_CHANGE_PERIOD, val));
    }

    //-- SYSTEM / CHAR COMMANDS -- (Must be last)
    else if (strcmp(msg, "sysreset") == 0) {
        // Reset the system
        SOAR_ASSERT(false, "System reset requested");
    }
    else if (strcmp(msg, "sysinfo") == 0) {
        // Print message
        SOAR_PRINT("\n\t-- SOAR System Info --\n");
        SOAR_PRINT("Current System Heap Use: %d Bytes\n", xPortGetFreeHeapSize());
        SOAR_PRINT("Lowest Ever Heap Size\t: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
        SOAR_PRINT("Debug Task Runtime  \t: %d ms\n\n", TICKS_TO_MS(xTaskGetTickCount()));
    }
    else if (strcmp(msg, "blinkled") == 0) {
        // Print message
        SOAR_PRINT("Debug 'LED blink' command requested\n");
        GPIO::LED1::On();
        // TODO: Send to HID task to blink LED, this shouldn't delay
    }
    else if (strcmp(msg, "baropoll") == 0) {
        // Send a request to the barometer task to poll the barometer
        SOAR_PRINT("Debug 'Barometer Poll' command requested\n");
        Command cmd(REQUEST_COMMAND, BARO_REQUEST_NEW_SAMPLE);
        BarometerTask::Inst().GetEventQueue()->Send(cmd);
    }
    else if (strcmp(msg, "baroread") == 0) {
        // Send a request to the barometer task to print the data
        SOAR_PRINT("Debug 'Barometer Read' command requested\n");
        Command cmd(REQUEST_COMMAND, BARO_REQUEST_DEBUG);
        BarometerTask::Inst().GetEventQueue()->Send(cmd);
    }
    else if (strcmp(msg, "barometer") == 0) {
        // Send a request to the barometer task to print the data
        SOAR_PRINT("Debug 'Barometer Poll+Read' command requested\n");
        Command cmd(REQUEST_COMMAND, BARO_REQUEST_NEW_SAMPLE);
        BarometerTask::Inst().GetEventQueue()->Send(cmd);
        Command cmd2(REQUEST_COMMAND, BARO_REQUEST_DEBUG);
        BarometerTask::Inst().GetEventQueue()->Send(cmd2);
    }
    else if (strcmp(msg, "imu") == 0) {
        // Send a request to the IMU task to poll and print the data
        SOAR_PRINT("Debug 'IMU Poll+Read' command requested\n");
        Command cmd(REQUEST_COMMAND, IMU_REQUEST_NEW_SAMPLE);
        IMUTask::Inst().GetEventQueue()->Send(cmd);
        Command cmd2(REQUEST_COMMAND, IMU_REQUEST_DEBUG);
        IMUTask::Inst().GetEventQueue()->Send(cmd2);
    }
    else if (strcmp(msg, "bat") == 0) {
 		SOAR_PRINT("Debug 'Battery Voltage' Sample and Output Received\n");
 		BatteryTask::Inst().SendCommand(Command(REQUEST_COMMAND, BATTERY_REQUEST_NEW_SAMPLE));
 		BatteryTask::Inst().SendCommand(Command(REQUEST_COMMAND, BATTERY_REQUEST_DEBUG));
 	}
    else if (strcmp(msg, "radiohb") == 0) {
        WatchdogTask::Inst().SendCommand(Command(HEARTBEAT_COMMAND, RADIOHB_REQUEST));
    }
    else if (strcmp(msg, "disablehb") == 0) {
        WatchdogTask::Inst().SendCommand(Command(HEARTBEAT_COMMAND, RADIOHB_DISABLED));
    }
    else if (strcmp(msg, "manualLaunch") == 0) {
    	TimerTransitions::Inst().ManualLaunch();
    }
    else if (strcmp(msg, "mev enable") == 0) {
    	GPIO::MEV_EN::On();
    }
    else if (strcmp(msg, "mev disable") == 0) {
    	GPIO::MEV_EN::Off();
    }
    else if (strcmp(msg, "mev close") == 0) {
    	PBBRxProtocolTask::SendPBBCommand(Proto::PBBCommand::Command::PBB_CLOSE_MEV);
    }
    else if (strcmp(msg, "mev open") == 0) {
    	//TODO: Remember to remove / make sure not enabled in final code
    	PBBRxProtocolTask::SendPBBCommand(Proto::PBBCommand::Command::PBB_OPEN_MEV);
    }
    else if (strcmp(msg, "ptc") == 0) {
		SOAR_PRINT("Debug 'Pressure Transducer' Sample and Output Received\n");
		PressureTransducerTask::Inst().SendCommand(Command(REQUEST_COMMAND, PT_REQUEST_NEW_SAMPLE));
		PressureTransducerTask::Inst().SendCommand(Command(REQUEST_COMMAND, PT_REQUEST_DEBUG));
	}
	else {
    }
    else if (strcmp(msg, "gps") == 0) {
    	GPSTask::Inst().SendCommand(Command(REQUEST_COMMAND, GPS_REQUEST_DEBUG));
    }
    else if (strcmp(msg, "gpstransmit") == 0) {
		GPSTask::Inst().SendCommand(Command(REQUEST_COMMAND, GPS_REQUEST_TRANSMIT));
	}
    else {
        // Single character command, or unknown command
        switch (msg[0]) {
        default:
            SOAR_PRINT("Debug, unknown command: %s\n", msg);
            break;
        }
    }

    //We've read the data, clear the buffer
    debugMsgIdx = 0;
    isDebugMsgReady = false;
}

/**
 * @brief Receive data, currently receives by arming interrupt
 */
bool DebugTask::ReceiveData()
{
    HAL_UART_Receive_IT(SystemHandles::UART_Debug, &debugRxChar, 1);
    return true;
}

/**
 * @brief Receive data to the buffer
 * @return Whether the debugBuffer is ready or not
 */
void DebugTask::InterruptRxData()
{
    // If we already have an unprocessed debug message, ignore this byte
    if (!isDebugMsgReady) {
        // Check byte for end of message - note if using termite you must turn on append CR
        if (debugRxChar == '\r' || debugMsgIdx == DEBUG_RX_BUFFER_SZ_BYTES) {
            // Null terminate and process
            debugBuffer[debugMsgIdx++] = '\0';
            isDebugMsgReady = true;

            // Notify the debug task
            Command cm(DATA_COMMAND, EVENT_DEBUG_RX_COMPLETE);
            bool res = qEvtQueue->SendFromISR(cm);

            // If we failed to send the event, we should reset the buffer, that way DebugTask doesn't stall
            if (res == false) {
                debugMsgIdx = 0;
                isDebugMsgReady = false;
            }
        }
        else {
            debugBuffer[debugMsgIdx++] = debugRxChar;
        }
    }

    //Re-arm the interrupt
    ReceiveData();
}

/* Helper Functions --------------------------------------------------------------*/
/**
 * @brief Extracts an integer parameter from a string
 * @brief msg Message to extract from, MUST be at least identifierLen long, and properly null terminated
 * @brief identifierLen Length of the identifier eg. 'rsc ' (Including the space) is 4
 * @return ERRVAL on failure, otherwise the extracted value
 */
int32_t DebugTask::ExtractIntParameter(const char* msg, uint16_t identifierLen)
{
    // Handle a command with an int parameter at the end
    if (static_cast<uint16_t>(strlen(msg)) < identifierLen+1) {
        SOAR_PRINT("Int parameter command insufficient length\r\n");
        return ERRVAL;
    }
    
    // Extract the value and attempt conversion to integer
    const int32_t val = Utils::stringToLong(&msg[identifierLen]);
    if (val == ERRVAL) {
        SOAR_PRINT("Int parameter command invalid value\r\n");
    }

    return val;
}
