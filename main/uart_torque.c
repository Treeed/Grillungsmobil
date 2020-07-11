//
// Created by treed on 26.06.2020.
//


#include <esp_log.h>
#include <driver/uart.h>
#include <stdio.h>

#include "error_handler.h"
#include "uart_torque.h"
#include "tandem_util.h"

QueueHandle_t torque_value_queue;

const uart_port_t uart_torque_num = UART_NUM_2;
const int msg_len = 10;
const int uart_torque_buffer_size = (128 * 2);
QueueHandle_t uart_event_queue;

bool process_torque_data(uint8_t* torque_data);

_Noreturn void uart_torque_rx_task(__unused void *pvParameters){
    uart_event_t event;
    uint8_t* rx_data = (uint8_t *) malloc(msg_len);

    int error_counter = 0;

    uart_intr_config_t intr_config ={
            .intr_enable_mask = UART_BRK_DET_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_PARITY_ERR_INT_ENA_M |UART_RXFIFO_FULL_INT_ENA_M,
            .txfifo_empty_intr_thresh = 0,
            .rxfifo_full_thresh = 0,
            .rx_timeout_thresh = 0,
    };

    while(1) {

        //set intr config to copy every byte so we can find out which is the starting byte
        intr_config.rxfifo_full_thresh = 1;
        uart_intr_config(uart_torque_num, &intr_config);

        do{
            //one dummy read to synchronize
            uart_read_bytes(uart_torque_num, rx_data, 1, pdMS_TO_TICKS(portMAX_DELAY));
            uart_read_bytes(uart_torque_num, rx_data, msg_len, pdMS_TO_TICKS(portMAX_DELAY));
        }while (!process_torque_data(rx_data));

        //go back to only copy when a full 10 bytes arrived
        intr_config.rxfifo_full_thresh = 10;
        uart_intr_config(uart_torque_num, &intr_config);
        xQueueReset(uart_event_queue);

        while (1) {
            xQueueReceive(uart_event_queue, (void *) &event, (portTickType) portMAX_DELAY);

            if (event.type != UART_DATA) {
                ESP_LOGI("torque_rx_event", "Uart event type: %d", event.type);
                error_counter += 100;
                continue;
            }

            if (uart_read_bytes(uart_torque_num, rx_data, msg_len, 0) != msg_len) {
                ESP_LOGI("torque_rx_event", "Didn't get right no. bytes!");
                error_counter += 100;
                continue;
            }

            if (!process_torque_data(rx_data)) {
                ESP_LOGI("torque_rx_event", "checksum didn't match");
                error_counter += 100;
                break;
            }

            if (error_counter > 10000) {
                ESP_LOGE("torque_rx_event", "more than 100 errors in last 10000 events");
            }
            if (error_counter > 0) {
                error_counter--;
            }

        }
    }
}

bool process_torque_data(uint8_t* torque_data){

    for(int pos = 0; pos < msg_len; pos++) {
        torque_data[pos] = reverseBits(torque_data[pos]);
    }

    uint8_t checksum = 0;
    for(int pos = 1; pos<msg_len; pos++){
        checksum += torque_data[pos];
    }
    if(torque_data[0] != checksum){
        return false;
    }


    uint16_t torques[3] = {0};
    for(int torque_no = 0; torque_no < 3; torque_no++){
        torques[torque_no] = torque_data[torque_no*2+1] | (torque_data[torque_no*2+2] & 0b00000011)<<8;
    }
    uint8_t angle = torque_data[4]>>2 & 0b00011111;
    uint16_t rotation_time = torque_data[8] | (torque_data[9] & 0b00111111)<<8;
    bool measurement_done = (torque_data[9]>>7) != 0;



    int32_t norm_torques[3] = {0};

    norm_torques[0] = torques[0] - 237;
    norm_torques[1] = torques[1] - 222;
    norm_torques[2] = torques[2] - 220;
    int32_t torque = MAX(MAX(norm_torques[0], norm_torques[1]), norm_torques[2]);

    xQueueOverwrite(torque_value_queue, &torque);


//    printf("%04u ", torques[0]);
//    printf("%04u ", torques[1]);
//    printf("%04u\n", torques[2]);
    //printf("%03d\n", torque);
//    printf("%02u\n", angle*3);
//    printf("%03u ", torque_data[7]*2);
//    //printf("%05u ", rotation_time/6);
//    printf("%2u\n", (uint8_t)measurement_done*30);

    return true;
}

void initialize_torque_uart(){
    uart_config_t uart_torque_config = {
            .baud_rate = 31450,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_ODD,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .use_ref_tick = 0
    };
    ESP_ERROR_CHECK(
            uart_param_config(uart_torque_num, &uart_torque_config)
    );
    ESP_ERROR_CHECK(
            uart_set_pin(uart_torque_num, UART_PIN_NO_CHANGE, UART_NUM_2_RXD_DIRECT_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)
    );

    ESP_ERROR_CHECK(
            uart_driver_install(uart_torque_num, uart_torque_buffer_size, 0, 10, &uart_event_queue, 0)
    );

    torque_value_queue = xQueueCreate(1, 4);

    xTaskCreate(uart_torque_rx_task, "uart_torque_rx_task", 2048, NULL, 12, NULL);

}