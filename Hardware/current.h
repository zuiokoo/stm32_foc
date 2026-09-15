#ifndef MOTOR1_CURRENT_H
#define MOTOR1_CURRENT_H

#include <stdint.h>


typedef struct{
    float current_a;
    float current_b;
    float current_c;
    
    uint16_t offset_a_raw;
    uint16_t offset_b_raw;
    
    uint32_t offset_sum_a;
    uint32_t offset_sum_b;   
    uint16_t offset_sample_count;
    uint8_t offset_calibrated;
    
}motor1_currentsense_t;


void motor1_current_init(motor1_currentsense_t *cs,uint16_t offset_a_raw,uint16_t offset_b_raw);
void motor1_currentsense_update(motor1_currentsense_t *cs,uint16_t adc_a_raw,uint16_t adc_b_raw);

void motor1_currentsense_calibration_start(motor1_currentsense_t*cs);
uint8_t motor1_currentsense_calibration_sample(motor1_currentsense_t*cs,uint16_t adc_a_raw,uint16_t adc_b_raw);

#endif

