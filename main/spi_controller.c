//
// Created by treed on 26.06.2020.
//

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <string.h>
#include "spi_controller.h"
#include "tandem_util.h"
#include "error_handler.h"
#include "motor_control.h"

spi_device_handle_t shift_out_register_handle;
spi_device_handle_t shift_in_register_handle;
QueueHandle_t shift_out_queue;

typedef struct {
    bool stay_on : 1;
    bool beeper : 1;
    bool led_blue : 1;
    bool led_green : 1;
    bool led_red : 1;
    unsigned int analog_mux_pin : 3;
} shift_out_bits;

typedef struct {
    unsigned int unused1 : 3;
    bool on_button : 1;
    bool support_pos_5 : 1;
    bool support_pos_4 : 1;
    bool support_pos_3 : 1;
    bool support_pos_2 : 1;
    unsigned int unused2 : 2;
    uint8_t check_bits : 2;
    bool support_pos_1 : 1;
    bool support_pos_0 : 1;
    unsigned int unused3 : 2;
} shift_in_bits;

void process_support_selector_switch(shift_in_bits values);

_Noreturn void shift_out_task(void *pvParameters){
    spi_transaction_t shift_out_transaction = {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .length = 8,
            .rxlength = 0,
            .user = 0,
            .tx_buffer = 0,
            .rx_buffer = 0
    };

    shift_out_bits shift_out_value = {
            .stay_on = 1,
            .led_red = 1,
            .led_green = 1,
            .led_blue = 1
    };

    while(1){
        ShiftOutActionStruct shift_out_action;
        xQueueReceive(shift_out_queue, &shift_out_action, portMAX_DELAY);

        switch (shift_out_action.shift_out_action) {
            case SET_ANALOG_MUX:
                shift_out_value.analog_mux_pin = shift_out_action.value;
                break;

            case SET_LED:
                break;

            case SET_FET_ON:
                shift_out_value.stay_on = shift_out_action.value;
                break;

            case SET_BEEP_ON:
                shift_out_value.beeper = shift_out_action.value;
        }

        shift_out_transaction.tx_buffer = &shift_out_value;
        spi_device_polling_transmit(shift_out_register_handle, &shift_out_transaction);
    }
}

_Noreturn void shift_in_task(void *pvParameters){
    shift_in_bits shift_in_buffer;
    spi_transaction_t shift_in_transaction = {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .length = 16,
            .rxlength = 16,
            .user = 0,
            .tx_buffer = 0,
            .rx_buffer = &shift_in_buffer
    };

    TickType_t on_released = xTaskGetTickCount();

    int consecutive_shift_in_errors = 0;

    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));

        spi_device_acquire_bus(shift_in_register_handle, portMAX_DELAY);
        gpio_set_level(GPIO_NUM_22, 1);
        spi_device_polling_transmit(shift_in_register_handle, &shift_in_transaction);
        gpio_set_level(GPIO_NUM_22, 0);
        spi_device_release_bus(shift_in_register_handle);

        if(shift_in_buffer.check_bits != 0b01){
            consecutive_shift_in_errors += 1;
            if(consecutive_shift_in_errors > 1){
                ErrorType active_errors;
                xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
                if(!active_errors.shift_in_error) {
                    ErrorType error_type = {0};
                    error_type.shift_in_error = 1;
                    ErrorStruct error = {
                            .error_type = error_type,
                            .error_msg = "shift in check bits were damaged thrice. received"
                    };
                    strcat(error.error_msg, binString( *((uint16_t *) &shift_in_buffer)));

                    xQueueSend(error_queue, &error, 0);
                }
            }
            continue;
        } else{
            consecutive_shift_in_errors = 0;
        }

        if(!shift_in_buffer.on_button){
            if((xTaskGetTickCount()-on_released) > pdMS_TO_TICKS(200)){
                printf("button pushed, turning off! shift in was ");
                printBits(2, &shift_in_buffer);
                turn_off();
            }
            on_released = xTaskGetTickCount();
        }

        process_support_selector_switch(shift_in_buffer);

        printBits(2, &shift_in_buffer);
        printf("\n");
    }
}

void process_support_selector_switch(shift_in_bits values){

    int sum = values.support_pos_0 +
              values.support_pos_1 +
              values.support_pos_2 +
              values.support_pos_3 +
              values.support_pos_4 +
              values.support_pos_5;
    if(sum < 1){
        return;
    }
    if(sum > 1){
        ErrorType active_errors;
        xQueuePeek(active_errors_queue, &active_errors, portMAX_DELAY);
        if(!active_errors.shift_in_error) {
            ErrorType error_type = {0};
            error_type.shift_in_error = 1;
            ErrorStruct error = {
                    .error_type = error_type,
                    .error_msg = "more than one switch position is active. shift in was"
            };
            strcat(error.error_msg, binString( *((uint16_t *) &values)));

            xQueueSend(error_queue, &error, 0);
        }
        return;
    }

    SupportLevelStruct support_level = {0};

    if(values.support_pos_0){
        support_level.motor_on = 0;
        support_level.max_current = 0;
        support_level.threshold = 0;

    }else if (values.support_pos_1){
        support_level.motor_on = 1;
        support_level.max_current = 3000;
        support_level.threshold = 30;

    }else if (values.support_pos_2){
        support_level.motor_on = 1;
        support_level.max_current = 6000;
        support_level.threshold = 30;

    }else if (values.support_pos_3){
        support_level.motor_on = 1;
        support_level.max_current = 9000;
        support_level.threshold = 30;

    }else if (values.support_pos_4){
        support_level.motor_on = 0;
        support_level.max_current = 0;
        support_level.threshold = 0;

    }else if (values.support_pos_5){
        support_level.motor_on = 0;
        support_level.max_current = 0;
        support_level.threshold = 0;

    }
    xQueueOverwrite(support_level_queue, &support_level);
}

void initialize_spi(){

    spi_bus_config_t spi_bus_config = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4094,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0
    };
    spi_bus_initialize(SPI2_HOST, &spi_bus_config, 0);


    spi_device_interface_config_t shift_out_register_conf = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 5,
        .clock_speed_hz = 10000000,
        .input_delay_ns = 0,
        .spics_io_num = 2,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
        .pre_cb = 0,
        .post_cb = 0
    };


    spi_bus_add_device(SPI2_HOST, &shift_out_register_conf, &shift_out_register_handle);
    spi_device_interface_config_t shift_in_register_conf = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 2,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = 5000000,
            .input_delay_ns = 0,
            .spics_io_num = -1,
            .flags = SPI_DEVICE_HALFDUPLEX,
            .queue_size = 1,
            .pre_cb = 0,
            .post_cb = 0
    };

    spi_bus_add_device(SPI2_HOST, &shift_in_register_conf, &shift_in_register_handle);

    gpio_config_t cs_shift_in_conf = {
            .pin_bit_mask = 1<<22,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cs_shift_in_conf);

    shift_out_queue = xQueueCreate(10, 8);
    xTaskCreate(shift_out_task, "shift_out_task", 1024, NULL, 11, NULL);
    xTaskCreate(shift_in_task, "shift_in_task", 4096, NULL, 10, NULL);
}