//
// Created by treed on 26.06.2020.
//

#include "esp_log.h"
#include <esp32/rom/uart.h>
#include <stdlib.h>

#include "tandem_util.h"



//dirty, blocking implementation, only for debugging
int int_from_serial(){
    const int strlen = 20;
    uint8_t myChar[strlen];
    UartRxString(myChar, strlen);
    char *end;
    return strtol((const char *) &myChar, &end, 10);
}

unsigned char reverseBits(unsigned char b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

int comparefunc (const void * a, const void * b)
{
    return ( *(int*)a - *(int*)b );
}