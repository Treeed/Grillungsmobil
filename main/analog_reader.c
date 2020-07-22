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

_Noreturn void read_analog_mux(void *pvParameters){
    esp_adc_cal_characteristics_t adc_characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc_characteristics);

    int battery_low_cnt = 0;
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        ShiftOutActionStruct set_mux_struct = {
                .shift_out_action = SET_ANALOG_MUX,
                .value = 5,
        };
        xQueueSend(shift_out_queue, &set_mux_struct, 0);

        const int avg_no = 100;

        uint32_t sum = 0;

        for (int read_no = 0; read_no < avg_no; read_no++) {
            sum += adc1_get_raw(ADC1_CHANNEL_3);
        }
        uint32_t avg = sum/avg_no;

        uint32_t voltage = avg*4.584+28824; //accurate to within 20mV up to 42 V directly after cal



        if(voltage < 35000){
            battery_low_cnt++;
            if(battery_low_cnt > 30) {

                ErrorType active_errors;
                xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
                if(!active_errors.battery_empty) {
                    ErrorType error_type = {0};
                    error_type.battery_empty = 1;
                    ErrorStruct error = {
                            .error_type = error_type,
                            .error_msg = "battery voltage below threshold"
                    };
                    xQueueSend(error_queue, &error, 0);
                }
            }
        } else{
            battery_low_cnt = 0;
        }

        xQueueOverwrite(analog_value_queue, &voltage);

        //printf("%f\n", (avg*4.584+28824));

        //esp_adc_cal_raw_to_voltage(avg, &adc_characteristics)
    }
}

void initialize_read_analog(){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    analog_value_queue = xQueueCreate(1, 4);

    xTaskCreate(read_analog_mux, "read_analog_mux", 2048, NULL, 10, NULL);
}