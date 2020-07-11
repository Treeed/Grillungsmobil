//
// Created by treed on 26.06.2020.
//

#ifndef UNTITLED_MOTOR_CONTROL_H
#define UNTITLED_MOTOR_CONTROL_H

#include <freertos/task.h>

void initialize_motor_control();

extern TaskHandle_t motor_task;

#endif //UNTITLED_MOTOR_CONTROL_H
