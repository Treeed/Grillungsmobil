//
// Created by treed on 26.06.2020.
//


#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/dac.h>
#include <driver/uart.h>

#include "error_handler.h"
#include "motor_control.h"
#include "spi_controller.h"


QueueHandle_t error_queue;
QueueHandle_t active_errors_queue;
void kill_motor();


_Noreturn void error_handler(__unused void *pvParameters){
    ErrorType active_errors = {0};
    xQueueOverwrite(active_errors_queue, &active_errors);

    while (1){
        ErrorStruct error;
        xQueueReceive(error_queue, &error, portMAX_DELAY);

        if(error.error_type.torque_error){
            kill_motor();
            active_errors.torque_error = 1;
            xQueueOverwrite(active_errors_queue, &active_errors);
        }
        if(error.error_type.battery_empty){
            kill_motor();
            active_errors.battery_empty = 1;
            xQueueOverwrite(active_errors_queue, &active_errors);
        }

        printf("%s\n", error.error_msg);
    }
}

void turn_off(){
    kill_motor();
    vTaskDelay(pdMS_TO_TICKS(500));
    ShiftOutActionStruct turn_off_action = {
            .shift_out_action = SET_FET_ON,
            .value = 0,
    };
    xQueueSend(shift_out_queue, &turn_off_action, 0);
}

void kill_motor(){
    dac_output_voltage(DAC_CHANNEL_1, 0);
    vTaskSuspend(motor_task);
}

void initialize_error_handler(){
    active_errors_queue = xQueueCreate(1, 1);
    error_queue = xQueueCreate(1, 104);
    xTaskCreate(error_handler, "error_handler", 2048, NULL, configMAX_PRIORITIES -1, NULL);
}