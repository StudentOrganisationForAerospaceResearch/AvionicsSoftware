/**
 ******************************************************************************
 * File Name          : WatchdogTask.cpp
 * Description        : Primary Watchdog task, default task for the system.
 ******************************************************************************
*/
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "WatchdogTask.hpp"
#include "FlightTask.hpp"
#include "DMBProtocolTask.hpp"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint32_t HEARTBEAT_TIMER_PERIOD_MS = 20 * 60 * 1000;

/**
 * @brief Constructor for WatchdogTask
 */
WatchdogTask::WatchdogTask() : Task(WATCHDOG_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the WatchdogTask
 */
void WatchdogTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)WatchdogTask::RunTask,
            (const char*)"WatchdogTask",
            (uint16_t)WATCHDOG_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)WATCHDOG_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "WatchdogTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief This function is called if the heartbeat timer expires and it sends a command to ABORT the system
 * @param Automatically passes is timer handle for callback, should not be used
 */
void WatchdogTask::HeartbeatFailureCallback(TimerHandle_t rtTimerHandle)
{
    Timer::DefaultCallback(rtTimerHandle);
    FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_ANY_TO_ABORT));
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void WatchdogTask::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {
    case HEARTBEAT_COMMAND: {
        HandleHeartbeat(cm.GetTaskCommand());
    }
    case TASK_SPECIFIC_COMMAND: {
        if(cm.GetTaskCommand() == HB_STATUS_SEND) {
            SendHeartbeatStatus();
        }
        break;
    }
    case RADIOHB_CHANGE_PERIOD: {
        SOAR_PRINT("HB Period Changed to %d s\n", (cm.GetTaskCommand()));
        heartbeatTimer->ChangePeriodMsAndStart((cm.GetTaskCommand()*1000));
        break;
    }
    default:
        SOAR_PRINT("WatchdogTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

void WatchdogTask::HandleHeartbeat(uint16_t taskCommand)
{
    switch (taskCommand) {
    case RADIOHB_REQUEST:
        GPIO::LED2::Toggle();
        SOAR_PRINT("HEARTBEAT RECEIVED \n");
        heartbeatTimer->ResetTimerAndStart();
        break;
    case RADIOHB_DISABLED:
        SOAR_PRINT("HEARTBEAT DISABLED \n");
        heartbeatTimer->Stop();
        break;
    default:
        SOAR_PRINT("WatchdogTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief Instance Run loop for the Watchdog Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void WatchdogTask::Run(void * pvParams)
{
    uint32_t tempSecondCounter = 0; // TODO: Temporary counter, would normally be in HeartBeat task or HID Task, unless FlightTask is the HeartBeat task
    GPIO::LED1::Off();

    heartbeatTimer = new Timer(HeartbeatFailureCallback);
    heartbeatTimer->ChangePeriodMs(HEARTBEAT_TIMER_PERIOD_MS);
    heartbeatTimer->Start();

    while (1) {
        GPIO::LED3::On();
        osDelay(500);
        GPIO::LED3::Off();
        osDelay(500);

        //Every cycle, print something out (for testing)
        SOAR_PRINT("FlightTask::Run() - [%d] Seconds\n", tempSecondCounter++);

        Command cm;

        // Ingest the command queue, up to 5 commands
        uint8_t proced = 0;
        while (qEvtQueue->Receive(cm) && (proced < 5)) {
            HandleCommand(cm);
            ++proced;
        }

    }
}

/**
 * @brief Utility function to convert heartbeat state
 */
static Proto::HeartbeatState::TimerState ConvertTimerStateToProto(TimerState ts) {
    switch(ts) {
    case TimerState::UNINITIALIZED:
        return Proto::HeartbeatState::TimerState::UNINITIALIZED;
    case TimerState::COUNTING:
        return Proto::HeartbeatState::TimerState::COUNTING;
    case TimerState::PAUSED:
        return Proto::HeartbeatState::TimerState::PAUSED;
    case TimerState::COMPLETE:
        return Proto::HeartbeatState::TimerState::COMPLETE;
    default:
        return Proto::HeartbeatState::TimerState::UNINITIALIZED;
    }
}

/**
 * @brief Sends a heartbeat status control message to the ground station
 */
void WatchdogTask::SendHeartbeatStatus()
{
    // Generate a PROTOBUF message and send it to the Protocol Task
    Proto::ControlMessage msg;
    msg.set_source(Proto::Node::NODE_DMB);
    msg.set_target(Proto::Node::NODE_RCU);
    Proto::HeartbeatState hbs;
    hbs.set_timer_state(ConvertTimerStateToProto(heartbeatTimer->GetState()));
    hbs.set_timer_period(heartbeatTimer->GetPeriodMs());
    hbs.set_timer_remaining(heartbeatTimer->GetRemainingTimeMs());
    msg.set_hb_state(hbs);
    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    // Send the control message
    DMBProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_CONTROL);
}
