//
// Created by treed on 06.07.2020.
//

#include "speed_in.h"
#include <driver/adc.h>
#include <driver/mcpwm.h>

_Noreturn void capture_speeds(void *pvParameters){

    uint32_t last_motor_time = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
    while(1){
        uint32_t new_motor_time = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
        if(new_motor_time == last_motor_time) {
            continue;
        }

        uint32_t motor_period = new_motor_time - last_motor_time;
        last_motor_time = new_motor_time;
        double motor_speed = 80000000.0/ (double) motor_period;
        printf("%f ", motor_speed);
    }
}

void initialize_speed_in(){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, 34);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
}