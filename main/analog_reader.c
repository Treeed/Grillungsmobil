//
// Created by treed on 30.06.2020.
//

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/adc.h>
#include <esp_log.h>
#include <esp_adc_cal.h>
#include "analog_reader.h"
#include "spi_controller.h"
#include "error_handler.h"

QueueHandle_t analog_value_queue;

void set_mux(uint8_t value);
uint32_t get_analog();
uint32_t calc_battery_voltage(uint32_t adc_val);
void check_battery_voltage(uint32_t voltage);

#define MUX_PIN_BATTERY_VOLT 5

_Noreturn void read_analog_mux(void *pvParameters){
    esp_adc_cal_characteristics_t adc_characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc_characteristics);


    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
        set_mux(MUX_PIN_BATTERY_VOLT);
        uint32_t avg = get_analog();
        uint32_t bat_voltage = calc_battery_voltage(avg);
        check_battery_voltage(bat_voltage);


        printf("%u bat volt \n", bat_voltage);

        xQueueOverwrite(analog_value_queue, &bat_voltage);

        //esp_adc_cal_raw_to_voltage(avg, &adc_characteristics)
    }
}

void set_mux(uint8_t value){
    ShiftOutActionStruct set_mux_struct = {
            .shift_out_action = SET_ANALOG_MUX,
            .value = value,
    };
    xQueueSend(shift_out_queue, &set_mux_struct, 0);
}

uint32_t get_analog(){
    const int avg_no = 100;

    uint32_t sum = 0;

    for (int read_no = 0; read_no < avg_no; read_no++) {
        sum += adc1_get_raw(ADC1_CHANNEL_3);
    }
    return sum/avg_no;
}

uint32_t calc_battery_voltage(uint32_t adc_val){
    return adc_val*4.584+28824; //accurate to within 20mV up to 42 V directly after cal
}




void check_battery_voltage(uint32_t voltage_mv){
    static int battery_low_cnt = 0;
    static int battery_empty_cnt = 0;

    if(voltage_mv < 33000){
        battery_low_cnt++;
        if(battery_low_cnt > 30) {

            ErrorType active_errors;
            xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
            if(!active_errors.battery_low) {
                ErrorType error_type = {0};
                error_type.battery_low = 1;
                ErrorStruct error = {
                        .error_type = error_type,
                        .error_msg = "battery voltage low"
                };
                xQueueSend(error_queue, &error, 0);
            }
        }
    } else{
        battery_low_cnt = 0;
    }
    if(voltage_mv < 31000){
        battery_empty_cnt++;
        if(battery_empty_cnt > 30) {

            ErrorType active_errors;
            xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
            if(!active_errors.battery_empty) {
                ErrorType error_type = {0};
                error_type.battery_empty = 1;
                ErrorStruct error = {
                        .error_type = error_type,
                        .error_msg = "battery empty, turning off"
                };
                xQueueSend(error_queue, &error, 0);
            }
        }
    } else{
        battery_empty_cnt = 0;
    }
}

void initialize_read_analog(){
    adc1_config_width(ADC_WIDTH_BIT_12);

    analog_value_queue = xQueueCreate(1, 4);

    xTaskCreate(read_analog_mux, "read_analog_mux", 4096, NULL, 10, NULL);
}