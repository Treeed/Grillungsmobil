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
void regulate_current(int32_t target);
double calc_amps(uint32_t raw_current);

_Noreturn static void motor_output_duty(__unused void *pvParameters){
    dac_output_enable(DAC_CHANNEL_1);
    vTaskDelay(pdMS_TO_TICKS(500));



    TickType_t last_torque = 0;
    #define torque_values_size 100
    int32_t torque_values[torque_values_size] = {0};
    uint16_t current_torque_pos = 0;
    int32_t target_current = 0;

    TickType_t loop_start = xTaskGetTickCount();
    while(1){
        if(xQueueReceive(torque_value_queue, &(torque_values[(current_torque_pos + 1) % torque_values_size]), 0) == pdTRUE){
            current_torque_pos = (current_torque_pos + 1) % torque_values_size;
            last_torque = xTaskGetTickCount();
        }
        if(xTaskGetTickCount() - last_torque > pdMS_TO_TICKS(100)){
            invoke_torque_error();
        }


        if(torque_values[current_torque_pos] > 5) {

            int32_t max_torque = torque_values[0];
            for(int torque_pos = 1; torque_pos < torque_values_size; torque_pos++){
                if(max_torque < torque_values[torque_pos]){
                    max_torque = torque_values[torque_pos];
                }
            }
            if(max_torque < 150) {
                max_torque = 0;
            }
            target_current = max_torque*30;
        } else{
            target_current = -1;
        }

//        printf("torque:%d ", torque_values[current_torque_pos]);

//        int target;
//        if(gpio_input_get() & 0b1) {
//            target_current = 0;
//        } else{
//            target_current = 6000;
//        }

        regulate_current(target_current);

        vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(5));
    }
}


void regulate_current(int32_t target){
    //ziegler nichols: schwingt bei Kp = 0.015
    //periode 70 zyklen
    //kp=0.00675 tn = 59.5 Ki =0.000113

    //ziegler nichols: schwingt bei Kp = 0.006
    //periode 17 zyklen
    //kp=0.0027 tn = 14.45 Ki = 0.000187

    static double i = 0;
    double output_value;

    double raw_current = calc_amps(get_current());
    double error_value = target-raw_current;

    double p = error_value * 0.001;
    i = MIN(255, MAX(5, error_value * 0.0001 + i));

    if(target > 0) {
        output_value = MIN(255, MAX(5, p+i));
    } else{
        output_value = 0;
    }

//    printf("p:%f ", p);
//    printf("i:%f ", i);
//    printf("cur:%f ", raw_current/30);
//    printf("target:%d ", target/30);
//    printf("out:%d", (uint8_t) output_value);
    printf("\n");

    dac_output_voltage(DAC_CHANNEL_1, (uint8_t) output_value);
}

double calc_amps(uint32_t raw_current){
    return (double)raw_current*5.33 + 154.84;
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
    xTaskCreate(motor_output_duty, "motor_output_duty", 10000, NULL, 13, &motor_task);

    //requires analog_reader to config adc peripheral first
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    //debug input
    gpio_config_t in0_config  = {
            .pin_bit_mask = 0b1,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in0_config);

    //for debugging, do not use with motor_output_duty
    //xTaskCreate(motor_duty_from_serial, "duty_from_serial", 2048, NULL, 13, NULL);
}