/*
 *  RunInterface.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: Chris (cjchanx)
 */

#include "main_avionics.hpp"
#include "UARTDriver.hpp"

extern "C" {
    void run_interface()
    {
        run_main();
    }

    void cpp_USART1_IRQHandler()
    {
        Driver::uart1.HandleIRQ_UART();
    }

    void cpp_USART2_IRQHandler()
    {
        Driver::uart2.HandleIRQ_UART();
    }

    void cpp_USART3_IRQHandler()
    {
        Driver::uart3.HandleIRQ_UART();
    }

    void cpp_USART5_IRQHandler()
    {
        Driver::uart5.HandleIRQ_UART();
    }

    void cpp_DMA1_Stream0_IRQHandler() {
        /* Check half-transfer complete interrupt */
        if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_0) && LL_DMA_IsActiveFlag_HT0(DMA1)) {
            LL_DMA_ClearFlag_HT0(DMA1);             /* Clear half-transfer complete flag */

        }

        /* Check transfer-complete interrupt */
        if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_0) && LL_DMA_IsActiveFlag_TC0(DMA1)) {
            LL_DMA_ClearFlag_TC0(DMA1);             /* Clear transfer complete flag */

        }
        //SOAR_PRINT("PLEASE\n");

    }

    void cpp_DMA1_Stream7_IRQHandler() {
        /* Check half-transfer complete interrupt */
        if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_7) && LL_DMA_IsActiveFlag_HT7(DMA1)) {
            LL_DMA_ClearFlag_HT7(DMA1);             /* Clear half-transfer complete flag */

        }

        /* Check transfer-complete interrupt */
        if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_7) && LL_DMA_IsActiveFlag_TC7(DMA1)) {
            LL_DMA_ClearFlag_TC7(DMA1);             /* Clear transfer complete flag */
            Driver::uart5.FinishDMA();
            LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);

        }


        //SOAR_PRINT("PLEASE\n");

    }
}



