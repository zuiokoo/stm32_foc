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
/* ---- 反馈滑动平均（N 个 PWM 周期，每个周期采一次）----
 * 为什么需要：采样链的分辨率就是 1 ADC LSB = 3.3V/4095/0.25V/A = 3.22 mA
 * （约 0.5A 给定的 0.64%），而且实测原始读数还有 3~6 LSB 的抖动
 * （13 帧锁轴数据：ia 在 56~66 个计数之间跳）。这噪声不但显示难看，
 * 更糟的是会被电流环放大成"真实"的电流摆动：kp 直接把它变成电压抖动，
 * ki 还会积分随机游走（实测 ki=10000 时 5ms 内积分漂 0.047V，和观察到的
 * vq 抖动 0.056V 吻合），于是 vq 出现 1.62~1.85V(13%) 的摆幅，
 * 真实电流跟着晃 ±0.02A —— 而这时转子是**固定**的、也**没有限幅**。
 * N=4：噪声 ÷2，群延迟 (N-1)/2×50us = 75us，在 200Hz 环路带宽处只滞后 5.4°。
 * N=8：噪声 ÷2.83，滞后 12.6°（还想更平滑就改这里，代价是相位）。
 * 滤波做在 ia/ib/ic 上（噪声进来的地方），Clarke/Park 拿到的是干净矢量。 */
#define CURRENT_LPF_N  8

typedef struct{
    float current_a;        /* 原始值（未滤波），VOFA 想看真实噪声时用它 */
    float current_b;
    float current_c;

    float current_a_f;      /* 滤波后：电流环实际使用 */
    float current_b_f;
    float current_c_f;
    float lpf_buf[3][CURRENT_LPF_N];
    uint8_t lpf_idx;
    uint8_t lpf_primed;

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

