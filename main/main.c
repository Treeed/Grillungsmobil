#include "uart_torque.h"
#include "error_handler.h"
#include "motor_control.h"
#include "spi_controller.h"
#include "analog_reader.h"
#include "speed_in.h"
#include "ble_tranceiver.h"

void app_main() {
    initialize_error_handler();
    initialize_torque_uart();
    initialize_motor_control();
    initialize_spi();
    initialize_read_analog();
    initialize_speed_in();
    //initialize_ble();
}

