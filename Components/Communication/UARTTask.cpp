/**
 ******************************************************************************
 * File Name          : UARTTask.cpp
 * Description        : UART
 ******************************************************************************
*/

#include "UARTTask.hpp"
#include "UARTDriver.hpp"
#include <cstring>
#include "cmsis_os.h"

/**
 * TODO: Currently not used, would be used for DMA buffer configuration or interrupt setup
 * @brief Configures UART DMA buffers and interrupts
 * 
*/


void UARTTask::ConfigureUART()
{
    // UART 5 - Uses polling for now (switch to DMA or interrupts once SOAR-Protocol is defined)
	// DMA IS BEGINNING WOW!!!!!!!!!!!

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)DMABUF);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, DEBUG_DMA_RX_BUF_SIZE);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_7, (uint32_t)TXBUF);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, DEBUG_DMA_TX_BUF_SIZE);

	  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_0);
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

	  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_7);
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
	 // HAL_UART_Receive_DMA(huart4, DMABUF, 40);


//	  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
//	  NVIC_EnableIRQ(DMA1_Stream0_IRQn);


	LL_USART_EnableIT_IDLE(UART5);



	  LL_USART_EnableDMAReq_RX(UART5);
	  LL_USART_EnableDMAReq_TX(UART5);


	    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_USART_DMA_GetRegAddr(UART5));
	    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(UART5));



	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);


	// WHY
	LL_USART_ClearFlag_FE(UART5);
	LL_USART_ClearFlag_IDLE(UART5);
	LL_USART_ClearFlag_LBD(UART5);
	LL_USART_ClearFlag_NE(UART5);
	LL_USART_ClearFlag_ORE(UART5);
	LL_USART_ClearFlag_PE(UART5);
	LL_USART_ClearFlag_RXNE(UART5);
	LL_USART_ClearFlag_TC(UART5);
	LL_USART_ClearFlag_nCTS(UART5);

	LL_DMA_ClearFlag_DME4(DMA1);
	LL_DMA_ClearFlag_FE4(DMA1);
	LL_DMA_ClearFlag_HT4(DMA1);
	LL_DMA_ClearFlag_TC4(DMA1);
	LL_DMA_ClearFlag_TE4(DMA1);



}



/**
 * @brief Initializes UART task with the RTOS scheduler
*/
void UARTTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize UART task twice");
    
    DMABUF = new uint8_t[DEBUG_DMA_RX_BUF_SIZE];
    memset(DMABUF,0xab,DEBUG_DMA_RX_BUF_SIZE);

    TXBUF = new uint8_t[DEBUG_DMA_TX_BUF_SIZE];
    memset(TXBUF,0xbc,DEBUG_DMA_TX_BUF_SIZE);

    strcpy((char*)TXBUF,"hello world");
    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)UARTTask::RunTask,
            (const char*)"UARTTask",
            (uint16_t)UART_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)UART_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "UARTTask::InitTask() - xTaskCreate() failed");

    // Configure DMA
    ConfigureUART();
     
}

/**
 * @brief Instance Run loop for the UART Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
*/
void UARTTask::Run(void * pvParams)
{
    //UART Task loop
    while(1) {
        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);
        
        //Process the command
        HandleCommand(cm);

    }
}

/**
 * @brief HandleCommand handles any command passed to the UART task primary event queue. Responsible for
 *           handling all commands, even if unsupported. (Unexpected commands must still be reset) 
 * @param cm Reference to the command object to handle
*/
void UARTTask::HandleCommand(Command& cm)
{
    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case DATA_COMMAND: {
        //Switch for task specific command within DATA_COMMAND
        switch (cm.GetTaskCommand()) {
        case UART_TASK_COMMAND_SEND_DEBUG:
            UART::Debug->Transmit(cm.GetDataPointer(), cm.GetDataSize());
            osDelay(1);
            break;
        case UART_TASK_COMMAND_SEND_RADIO:
            UART::Radio->Transmit(cm.GetDataPointer(), cm.GetDataSize());
            osDelay(1);
        	break;
        case UART_TASK_COMMAND_SEND_PBB:
            UART::Conduit_PBB->Transmit(cm.GetDataPointer(), cm.GetDataSize());
            osDelay(1);
            break;
        default:
            SOAR_PRINT("UARTTask - Received Unsupported DATA_COMMAND {%d}\n", cm.GetTaskCommand());
            break;
        }
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    default:
        SOAR_PRINT("UARTTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}
