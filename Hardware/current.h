#ifndef MOTOR2_CURRENT_H
#define MOTOR2_CURRENT_H

#include <stdint.h>


/* 通道与相的对应关系（已按实际接线确认）：
 *   current_a ← 注入组 rank1(CHANNEL_5/PA5) ← TIM1_CH1(PA8) 所驱动那一相
 *   current_b ← 注入组 rank2(CHANNEL_4/PA4) ← TIM1_CH2(PA9) 所驱动那一相
 * foc_clarke_transform() 按这个约定解析。对调 rank1/rank2 只会把两相变成镜像
 * （id/iq 里混入旋转分量），并不能改变正负号。若改动这里，adc.c 的注入通道顺序
 * 和 stm32_foc.ioc 必须同时改，并重做相序与电流方向验证：
 * 开环 θ=0、Vq>0 时 a 相读数应为 0、b 相为正、c 相为负。
 * 符号约定：电流流入电机为正，由 CURRENT_SENSE_SIGN 与采样链极性的乘积决定。 */
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

