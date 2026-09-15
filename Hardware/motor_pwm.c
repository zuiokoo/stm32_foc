#include "motor_pwm.h"
#include "tim.h"

HAL_StatusTypeDef motor1_pwm_start(void){

    motor1_pwm_set_duty(0.5f, 0.5f, 0.5f);
    if(HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1)!=HAL_OK){
        motor1_pwm_stop();
        return HAL_ERROR;
    }
    if(HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2)!=HAL_OK){
        motor1_pwm_stop();
        return HAL_ERROR;
    }
    if(HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3)!=HAL_OK){
        motor1_pwm_stop();
        return HAL_ERROR;
    }
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
void motor1_pwm_set_duty(float duty_u,float duty_v,float duty_w){


      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty_to_compare(duty_u));
      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty_to_compare(duty_v));
      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty_to_compare(duty_w));


}

void motor1_pwm_stop(void){

     __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0);
     __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, 0);
     __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, 0);
    
     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); 
    
     __HAL_TIM_MOE_DISABLE(&htim1);
}



