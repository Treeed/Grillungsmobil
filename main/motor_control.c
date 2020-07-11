#include <soc.h>
//
// Created by treed on 26.06.2020.
//

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/dac.h>
#include <driver/uart.h>
#include <stdio.h>
#include <driver/adc.h>
#include <driver/mcpwm.h>

#include "tandem_util.h"
#include "motor_control.h"
#include "error_handler.h"
#include "uart_torque.h"

TaskHandle_t motor_task;

void invoke_torque_error();

uint32_t get_current();

_Noreturn static void motor_output_duty(__unused void *pvParameters){
    dac_output_enable(DAC_CHANNEL_1);
    const int no_init_samples = 100;
    const int init_time_ms = 2000;
    uint32_t sum = 0;

    TickType_t start_time = xTaskGetTickCount();
    for(int sample_no = 0; sample_no < no_init_samples; sample_no++){
        sum += get_current();
        vTaskDelayUntil(&start_time, pdMS_TO_TICKS(init_time_ms/no_init_samples));
    }
    uint32_t current_offset = sum / no_init_samples;


    int last_error = 0;
    double output_value = 0;
    TickType_t last_torque = 0;
    while(1){
        int32_t torque_value;
        if(xQueueReceive(torque_value_queue, &torque_value, 0) == pdTRUE){
            last_torque = xTaskGetTickCount();
        }
        if(xTaskGetTickCount() - last_torque > pdMS_TO_TICKS(100)){
            invoke_torque_error();
        }

        int target = (torque_value-5)*20;

        int raw_current = get_current();
        raw_current = MAX(raw_current - (int) current_offset, 0);
        int error_value = target-raw_current;

        if(target > 0) {
            output_value = MIN(255, MAX(5, (
                    output_value + error_value * 0.001 + (error_value - last_error) * 0
            )));
        } else{
            output_value = 0;
        }


        printf("%d ", raw_current);
        printf("%d ", target);
        printf("%f\n", output_value);
        dac_output_voltage(DAC_CHANNEL_1, (uint8_t) output_value);
    }
}

uint32_t get_current() {
    const int sampleLen = 20;
    uint32_t vals[sampleLen];
    for (int read_no = 0; read_no < sampleLen/3; read_no++) {
        vals[read_no] = adc1_get_raw(ADC1_CHANNEL_0);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    for (int read_no = sampleLen/3; read_no < sampleLen/3*2; read_no++) {
        vals[read_no] = adc1_get_raw(ADC1_CHANNEL_0);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    for (int read_no = sampleLen/3*2; read_no < sampleLen; read_no++) {
        vals[read_no] = adc1_get_raw(ADC1_CHANNEL_0);
    }

    qsort(vals, sampleLen, 4, comparefunc);
    return vals[sampleLen/2];
}

void invoke_torque_error() {
    ErrorType active_errors;
    xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
    if(!active_errors.torque_error) {
        ErrorType error_type = {0};
        error_type.torque_error = 1;
        ErrorStruct error = {
                .error_type = error_type,
                .error_msg = "no torque values within last 100ms, shutting down motor"
        };
        xQueueSend(error_queue, (void *) &error, portMAX_DELAY);
    }
}

_Noreturn void motor_duty_from_serial(__unused void *pvParameters){
    dac_output_enable(DAC_CHANNEL_1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    while(1){
        int serial_read_value = int_from_serial();
        printf("%d", serial_read_value);
        dac_output_voltage(DAC_CHANNEL_1, serial_read_value);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void initialize_motor_control(){
    xTaskCreate(motor_output_duty, "motor_output_duty", 2048, NULL, 13, &motor_task);

    //requires analog_reader to config adc peripheral first
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    //debug input
//    gpio_config_t in0_config  = {
//            .pin_bit_mask = 0b1,
//            .mode = GPIO_MODE_INPUT,
//            .pull_up_en = GPIO_PULLUP_DISABLE,
//            .pull_down_en = GPIO_PULLDOWN_DISABLE,
//            .intr_type = GPIO_INTR_DISABLE
//    };
//    gpio_config(&in0_config);

    //for debugging, do not use with motor_output_duty
    //xTaskCreate(motor_duty_from_serial, "duty_from_serial", 2048, NULL, 13, NULL);
}