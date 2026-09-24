#include "motor_pwm.h"
#include "tim.h"

/* 注意：这里刻意不用 HAL_TIM_PWM_Start/Stop，而是直接操作寄存器宏。
 * 原因：F4 的 HAL 在这两个函数里会检查"通道状态"(HAL_TIM_CHANNEL_STATE_READY)，
 * 一旦因为反复 start/stop/fault 导致簿记与硬件不一致，HAL_TIM_PWM_Start 会
 * 直接返回 HAL_ERROR 且不使能任何输出 —— 表现就是 "RUN: pwm start fail" + FAULT，
 * PWM 永远起不来（本工程实际踩过）。
 * 直接写 CCER 位 + MOE 没有这个状态机，行为与硬件一致、可重复。 */
HAL_StatusTypeDef motor2_pwm_start(void){

    motor2_pwm_set_duty(0.5f, 0.5f, 0.5f);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
    __HAL_TIM_MOE_ENABLE(&htim1);      /* TIM1 是带 break 的高级定时器，必须开 MOE 才有输出 */
    __HAL_TIM_ENABLE(&htim1);          /* 计数器运行（ADC 注入组的 CC4 触发也依赖它） */
    return HAL_OK;
}

static uint32_t duty_to_compare(float duty){

      uint32_t period;
      period=  __HAL_TIM_GET_AUTORELOAD(&htim1);
      if (duty < 0.0f)
      {
        duty = 0.0f;
      }
      if (duty > 1.0f)
      {
        duty = 1.0f;
      }
      return (uint32_t)(duty * (float)period);

}
void motor2_pwm_set_duty(float duty_u,float duty_v,float duty_w){


      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty_to_compare(duty_u));
      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty_to_compare(duty_v));
      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty_to_compare(duty_w));


}

void motor2_pwm_stop(void){

     /* 先把占空比归零（三路输出立即变低），再关通道、最后关 MOE —— 顺序保证不会出现半桥直通 */
     __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0);
     __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, 0);
     __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, 0);

     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);

     __HAL_TIM_MOE_DISABLE(&htim1);
}



