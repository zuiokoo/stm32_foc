
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef motor1_pwm_start(void);

void motor1_pwm_set_duty(float duty_u,float duty_v,float duty_w);

void motor1_pwm_stop(void);
      

#endif
