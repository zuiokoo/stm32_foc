#include "current.h"
#define ADC_VREF        3.3f
#define ADC_MAX_COUNT   4095.0f
#define SHUNT_RESISTOR  0.005f
#define AMP_GAIN        50.0f
#define CURRENT_OFFSET_SAMPLE_COUNT 1000U


void motor2_current_init(motor2_currentsense_t *cs,uint16_t offset_a_raw,uint16_t offset_b_raw){

    cs->current_a=0.0f;
    cs->current_b=0.0f;
    cs->current_c=0.0f;
    
    cs->offset_a_raw=offset_a_raw;
    cs->offset_b_raw=offset_b_raw;
    
}
void motor2_currentsense_update(motor2_currentsense_t *cs,uint16_t adc_a_raw,uint16_t adc_b_raw){
    
    cs->current_a= (ADC_VREF*(adc_a_raw - cs->offset_a_raw)/ADC_MAX_COUNT)/(SHUNT_RESISTOR*AMP_GAIN);
    cs->current_b= (ADC_VREF*(adc_b_raw - cs->offset_b_raw)/ADC_MAX_COUNT)/(SHUNT_RESISTOR*AMP_GAIN);
    cs->current_c= - cs->current_a - cs->current_b;
}


void motor2_currentsense_calibration_start(motor2_currentsense_t*cs){
    cs->offset_sum_a=0;
    cs->offset_sum_b=0;   
    cs->offset_sample_count=0;
    cs->offset_calibrated=0;

}
uint8_t motor2_currentsense_calibration_sample(motor2_currentsense_t*cs,uint16_t adc_a_raw,uint16_t adc_b_raw){
    
    if(cs->offset_calibrated==1){
        return 1;
    }
    
    cs->offset_sum_a+=adc_a_raw;
    cs->offset_sum_b+=adc_b_raw;
    cs->offset_sample_count++;
    if(cs->offset_sample_count>=CURRENT_OFFSET_SAMPLE_COUNT){
        cs->offset_calibrated=1;
        cs->offset_a_raw=cs->offset_sum_a/CURRENT_OFFSET_SAMPLE_COUNT;
        cs->offset_b_raw=cs->offset_sum_b/CURRENT_OFFSET_SAMPLE_COUNT;
        return 1;
    }
    return 0;
    
}
