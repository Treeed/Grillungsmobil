//
// Created by treed on 26.06.2020.
//

#ifndef UNTITLED_ERROR_HANDLER_H
#define UNTITLED_ERROR_HANDLER_H

#include <freertos/queue.h>

typedef struct{
    bool torque_error : 1;
    bool battery_empty : 1;
} ErrorType; //increase queue size when this becomes > 8 values!

typedef struct{
    ErrorType error_type;
    char error_msg[100];
}ErrorStruct;



extern QueueHandle_t error_queue;
extern QueueHandle_t active_errors_queue;

void initialize_error_handler();

#endif //UNTITLED_ERROR_HANDLER_H
