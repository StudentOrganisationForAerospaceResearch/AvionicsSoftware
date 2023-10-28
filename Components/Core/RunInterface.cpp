/*
 *  RunInterface.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: Chris (cjchanx)
 */

#include "UARTDriver.hpp"
#include "main_avionics.hpp"

extern "C" {
void run_interface() {
    run_main();
}

void cpp_USART1_IRQHandler() {
    Driver::uart1.HandleIRQ_UART();
}

void cpp_USART2_IRQHandler() {
    Driver::uart2.HandleIRQ_UART();
}

void cpp_USART3_IRQHandler() {
    Driver::uart3.HandleIRQ_UART();
}

void cpp_USART5_IRQHandler() {
    Driver::uart5.HandleIRQ_UART();
}
}
