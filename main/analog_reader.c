//
// Created by treed on 30.06.2020.
//

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/adc.h>
#include <esp_log.h>
#include <esp_adc_cal.h>
#include <math.h>
#include "analog_reader.h"
#include "spi_controller.h"
#include "error_handler.h"

QueueHandle_t analog_value_queue;

void set_mux(uint8_t value);
uint32_t get_analog();
uint32_t calc_battery_voltage(uint32_t adc_val);
void check_battery_voltage(uint32_t voltage);
double calc_PTC_temp(uint32_t adc_volt_mv);
double calc_NTC_temp(uint32_t adc_volt_mv);

#define MUX_PIN_BATTERY_VOLT 5
#define MUX_PIN_ESC_TEMP 6
#define MUX_PIN_BUCK_TEMP 4
#define MUX_PIN_MOTOR_TEMP 2

_Noreturn void read_analog_mux(void *pvParameters){
    esp_adc_cal_characteristics_t adc_characteristics_11db;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc_characteristics_11db);
    esp_adc_cal_characteristics_t adc_characteristics_6db;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 0, &adc_characteristics_6db);


    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
        set_mux(MUX_PIN_BATTERY_VOLT);
        uint32_t avg = get_analog();
        uint32_t bat_voltage = calc_battery_voltage(avg);
        check_battery_voltage(bat_voltage);

        set_mux(MUX_PIN_ESC_TEMP);
        avg = get_analog();
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(avg, &adc_characteristics_11db);
        double esc_temp = calc_PTC_temp(voltage_mv);

        set_mux(MUX_PIN_BUCK_TEMP);
        avg = get_analog();
        voltage_mv = esp_adc_cal_raw_to_voltage(avg, &adc_characteristics_11db);
        double buck_temp = calc_PTC_temp(voltage_mv);

        adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
        set_mux(MUX_PIN_MOTOR_TEMP);
        avg = get_analog();
        voltage_mv = esp_adc_cal_raw_to_voltage(avg, &adc_characteristics_6db);
//        printf("volt %d\n", voltage_mv);
        double motor_temp = calc_NTC_temp(voltage_mv);

        if(esc_temp > 80 || buck_temp > 80 || motor_temp > 80){
            ErrorType active_errors;
            xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
            if(!active_errors.battery_empty) {
                ErrorType error_type = {0};
                error_type.battery_empty = 1;
                ErrorStruct error = {
                        .error_type = error_type,
                        .error_msg = "temp low"
                };
                xQueueSend(error_queue, &error, 0);
            }
        }
//        printf("bat:%f ", (float) bat_voltage/1000);
//        printf("esc:%f ",esc_temp);
//        printf("buck:%f ",buck_temp);
//        printf("motor:%f\n",motor_temp);

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

double calc_PTC_temp(uint32_t adc_volt_mv){
    //uses 407.7669Ohms resistor to ground and kty81 210 upper https://www.nxp.com/docs/en/data-sheet/KTY81_SER.pdf
    // 5/(x/407.8+1)=y x is resistance, y is voltage out
    // y = 18,308x + 1549,7  y is resistance, x is temp

    return 111372.0 / adc_volt_mv - 106.92;
}

double calc_NTC_temp(uint32_t adc_volt_mv){
    // -90/(z - 5) - 18=y z is voltage y is resistance
    // -26,65ln(x) + 269,59 = y x is resistance y is temp
    return -26.65*log( -18000.0 - 90000000.0/(-5000.0 + adc_volt_mv)) + 269.59;
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