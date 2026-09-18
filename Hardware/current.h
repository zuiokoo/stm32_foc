#ifndef MOTOR2_CURRENT_H
#define MOTOR2_CURRENT_H

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
    
}motor2_currentsense_t;


void motor2_current_init(motor2_currentsense_t *cs,uint16_t offset_a_raw,uint16_t offset_b_raw);
void motor2_currentsense_update(motor2_currentsense_t *cs,uint16_t adc_a_raw,uint16_t adc_b_raw);

void motor2_currentsense_calibration_start(motor2_currentsense_t*cs);
uint8_t motor2_currentsense_calibration_sample(motor2_currentsense_t*cs,uint16_t adc_a_raw,uint16_t adc_b_raw);

#endif

