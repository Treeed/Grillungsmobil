//
// Created by treed on 26.06.2020.
//

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include "spi_controller.h"
#include "tandem_util.h"

spi_device_handle_t shift_out_register_handle;
spi_device_handle_t shift_in_register_handle;
QueueHandle_t shift_out_queue;

typedef struct {
    bool stay_on : 1;
    unsigned int unused : 4;
    unsigned int analog_mux_pin : 3;
} shift_out_bits;

typedef struct {
    unsigned int unused1 : 3;
    bool on_button : 1;
    unsigned int unused2 : 12;
} shift_in_bits;


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
    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));

        spi_device_acquire_bus(shift_in_register_handle, portMAX_DELAY);
        gpio_set_level(GPIO_NUM_22, 1);
        spi_device_polling_transmit(shift_in_register_handle, &shift_in_transaction);
        gpio_set_level(GPIO_NUM_22, 0);
        spi_device_release_bus(shift_in_register_handle);

//
//        printBits(2, &shift_in_buffer);
//        printf("\n");
    }
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
    xTaskCreate(shift_out_task, "shift_out_task", 1024, NULL, 10, NULL);
    xTaskCreate(shift_in_task, "shift_in_task", 2048, NULL, 10, NULL);

}