//
// Created by treed on 26.06.2020.
//

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/queue.h>
#include "spi_controller.h"

spi_device_handle_t shift_out_register_handle;
QueueHandle_t shift_out_queue;

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
    uint8_t shift_out_value = 0b00000000;
    while(1){
        ShiftOutActionStruct shift_out_action;
        xQueueReceive(shift_out_queue, &shift_out_action, portMAX_DELAY);

        switch (shift_out_action.shift_out_action) {
            case SET_ANALOG_MUX:
                shift_out_value = (shift_out_value & 0b00011111) | shift_out_action.value << 5;
                break;

            case SET_LED:
                break;
        }

        shift_out_transaction.tx_buffer = &shift_out_value;
        spi_device_polling_transmit(shift_out_register_handle, &shift_out_transaction);
    }
}

void initialize_spi(){

    spi_bus_config_t spi_bus_config = {
        .mosi_io_num = 21,
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
        .cs_ena_posttrans = 15,
        .clock_speed_hz = 10000000,
        .input_delay_ns = 0,
        .spics_io_num = 2,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
        .pre_cb = 0,
        .post_cb = 0
    };


    spi_bus_add_device(SPI2_HOST, &shift_out_register_conf, &shift_out_register_handle);

    shift_out_queue = xQueueCreate(10, 8);
    xTaskCreate(shift_out_task, "shift_out_task", 1024, NULL, 10, NULL);

}