//
// Created by treed on 26.06.2020.
//

#ifndef UNTITLED_UART_TORQUE_H
#define UNTITLED_UART_TORQUE_H

#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>

extern QueueHandle_t torque_value_queue;

_Noreturn void uart_torque_rx_task(void *pvParameters);
void initialize_torque_uart();

#endif //UNTITLED_UART_TORQUE_H
