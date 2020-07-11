//
// Created by treed on 26.06.2020.
//

#ifndef UNTITLED_SPI_CONTROLLER_H
#define UNTITLED_SPI_CONTROLLER_H

typedef enum{
    SET_ANALOG_MUX,
    SET_LED,
} ShiftOutAction;

typedef struct {
    ShiftOutAction shift_out_action;
    uint8_t value;
} ShiftOutActionStruct;

extern QueueHandle_t shift_out_queue;

void initialize_spi();

#endif //UNTITLED_SPI_CONTROLLER_H
