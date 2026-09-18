#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

#define VOFA_DATA_NUM 7
typedef struct{
    volatile uint8_t vofa_enable;
    volatile uint8_t vofa_send_flag;
    float vofa_data[VOFA_DATA_NUM];
}vofa_t;

void vofa_init(vofa_t*vofa);
void vofa_send(float*data,uint8_t num);
void vofa_capture(float*data);

#endif
