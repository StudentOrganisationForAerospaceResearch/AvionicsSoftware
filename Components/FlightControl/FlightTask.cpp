/**
 ******************************************************************************
 * File Name          : FlightTask.cpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#include "FlightTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "DMBProtocolTask.hpp"
#include "TimerTransitions.hpp"

/**
 * @brief Constructor for FlightTask
 */
FlightTask::FlightTask() : Task(FLIGHT_TASK_QUEUE_DEPTH_OBJS)
{
    rsm_ = nullptr;
}

/**
 * @brief Initialize the FlightTask
 */
void FlightTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");
    
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FlightTask::RunTask,
            (const char*)"FlightTask",
            (uint16_t)FLIGHT_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)FLIGHT_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "FlightTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Flight Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlightTask::Run(void * pvParams)
{
	//TODO: This should probably be true for enter state, although this behavior is dictated by flash state recovery
    rsm_ = new RocketSM(RS_ABORT, false);
    TimerTransitions::Inst().Setup();
    while (1) {
        // There's effectively 3 types of tasks... 'Async' and 'Synchronous-Blocking' and 'Synchronous-Non-Blocking'
        // Asynchronous tasks don't require a fixed-delay and can simply delay using xQueueReceive, it will immedietly run the next task
        // cycle as soon as it gets an event.

        // Synchronous-Non-Blocking tasks require a fixed-delay and will require something like an RTOS timer that tracks the time till the next run cycle,
        // and will delay using xQueueReceive for the set time, but if it gets interrupted by an event will handle the event then restart a xQueueReceive with
        // the time remaining in the timer

        // Synchronous-Blocking tasks are simpler to implement, they do NOT require instant handling of queue events, and will simply delay using osDelay() and
        // poll the event queue once every cycle.

        // This task below with the display would be a 'Synchronous-Non-Blocking' we want to handle queue events instantly, but keep a fixed delay
        // Could consider a universal queue that directs and handles commands to specific tasks, and a task that handles the queue events and then calls the
        // Mappings between X command and P subscribers (tasks that are expecting it).

        // Since FlightTask is so critical to managing the system, it may make sense to make this a Async task that handles commands as they come in, and have these display commands be routed over to the DisplayTask
        // or maybe HID (Human Interface Device) task that handles both updating buzzer frequencies and LED states.


        //Process commands in blocking mode (TODO: Change to instant-processing once complete HID/DisplayTask)
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res)
            HandleCommand(cm);

        //osDelay(FLIGHT_PHASE_DISPLAY_FREQ);

        //// Half the buzzer frequency for flight phase beeps
        //// (slightly less important, and only a bit quieter)
        //htim2.Init.Prescaler = ((htim2.Init.Prescaler + 1) * 2) - 1;
        //if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        //    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        //    osDelay(BUZZER_ERR_PERIOD);
        //    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        //}

        //// Beep n times for flight phase n, and blink LED 1
        //for (int i = -1; i < getCurrentFlightPhase(); i++) {
        //    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        //    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 1);
        //    osDelay(FLIGHT_PHASE_BLINK_FREQ);

        //    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        //    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 0);
        //    osDelay(FLIGHT_PHASE_BLINK_FREQ);
        //}

        //// Return the buzzer to its optimal frequency for message beeps
        //htim2.Init.Prescaler = ((htim2.Init.Prescaler + 1) / 2) - 1;
        //if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        //    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        //    osDelay(BUZZER_ERR_PERIOD);
        //    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        //}

        // TODO: Message beeps
    }
}

/**
 * @brief Handle a command from the Command Queue
 * @param cm Command to handle
 */
void FlightTask::HandleCommand(Command& cm)
{
    // If this is a request command, we handle it in the task (rocket state command must always be control actions)
    if (cm.GetCommand() == REQUEST_COMMAND && cm.GetTaskCommand() == RocketState::RS_ABORT)
        SendRocketState();
    else
        rsm_->HandleCommand(cm);

	// Make sure the command is reset
    cm.Reset();
}

/**
 * @brief Sends rocket state commands to the RCU
 */
void FlightTask::SendRocketState()
{
    // For testing, generate a PROTOBUF message and send it to the Protocol Task
    Proto::ControlMessage msg;
    msg.set_source(Proto::Node::NODE_DMB);
    msg.set_target(Proto::Node::NODE_RCU);
    msg.set_message_id(Proto::MessageID::MSG_CONTROL);
    Proto::SystemState stateMsg;
    stateMsg.set_sys_state(Proto::SystemState::State::SYS_NORMAL_OPERATION);
    stateMsg.set_rocket_state(rsm_->GetRocketStateAsProto());
    msg.set_sys_state(stateMsg);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    // Send the control message
    DMBProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_CONTROL);
}
