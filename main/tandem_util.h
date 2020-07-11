//
// Created by treed on 26.06.2020.
//

#ifndef UNTITLED_TANDEM_UTIL_H
#define UNTITLED_TANDEM_UTIL_H

#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

int comparefunc (const void * a, const void * b);
unsigned char reverseBits(unsigned char b);
int int_from_serial();


#endif //UNTITLED_TANDEM_UTIL_H
