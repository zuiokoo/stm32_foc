/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "current.h"
#include "as5600.h"
#include "foc_math.h"
#include "pi_controller.h"
#include "motor_pwm.h"
#include "motor_cli.h"
#include "vofa.h"
#include <stdio.h>
#include "debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LED1_ON  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define LED1_OFF  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)
#define LED2_ON  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET)
#define LED2_OFF  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET)
#define MOTOR2_ALIGN_VOLTAGE 0.5f

typedef enum
{
    MOTOR2_STATE_CALIBRATING = 0,
    MOTOR2_STATE_ALIGN,
    MOTOR2_STATE_READY,
    MOTOR2_STATE_RUNNING,
    MOTOR2_STATE_OPENLOOP,
    MOTOR2_STATE_FAULT

} motor2_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



motor2_currentsense_t motor2_current;
as5600_t motor2_encoder;

float motor2_electrical_zero_offset_rad = 0.0f;
int8_t motor2_sensor_direction = 1;

float motor2_mechanical_angle_rad;
volatile float motor2_electrical_angle_rad;
uint32_t encoder_last_tick;
uint32_t now_tick;
static uint32_t led1_last_tick = 0;
static uint32_t motor2_angle_last_ok_tick = 0;
static uint32_t motor2_align_start_tick = 0U;   /* ���뿪ʼʱ�� */
static uint8_t  motor2_align_started    = 0U;   /* �Ƿ����������� */

foc_pi_t motor2_pi_d;
foc_pi_t motor2_pi_q;

volatile  float motor2_id_ref=0.0f;
volatile  float motor2_iq_ref=0.0f;

float motor2_i_alpha;
float motor2_i_beta;
float motor2_id;
float motor2_iq;

float motor2_vd;
float motor2_vq;
float motor2_v_alpha;
float motor2_v_beta;

float motor2_duty_u;
float motor2_duty_v;
float motor2_duty_w;

volatile uint8_t motor2_fault;
volatile uint8_t motor2_run_enable = 0;

volatile motor2_state_t motor2_state =MOTOR2_STATE_CALIBRATING;
static   motor2_state_t motor2_state_last=MOTOR2_STATE_CALIBRATING;
volatile uint8_t motor2_angle_valid = 0;

uint8_t uart2_rx_byte;
volatile uint8_t motor2_align_request=0;

vofa_t motor2_vofa;

uint8_t led1_state=0;

volatile float motor2_openloop_angle = 0.0f;      // 开环电角度
volatile float motor2_openloop_speed = 50.0f;      // 电角度速度 rad/s
volatile float motor2_openloop_voltage = 1.0f;    // 开环电压幅值
volatile uint8_t motor2_openloop_enable = 0;      // 开环使能
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  DBG("Hello FOC\r\n");

  vofa_init(&motor2_vofa);
  motor_cli_init();
  DBG("Init: cli ok, vofa_enable=%d\r\n", motor2_vofa.vofa_enable);
  HAL_UART_Receive_IT(&huart2,&uart2_rx_byte,1);
  as5600_init(&motor2_encoder, &hi2c1);

  encoder_last_tick = HAL_GetTick();
  motor2_current_init(&motor2_current,0,0);
  motor2_currentsense_calibration_start(&motor2_current);
  
  foc_pi_init(&motor2_pi_d,0.0f,0.0f,-6.0f,6.0f);
  foc_pi_init(&motor2_pi_q,0.0f,0.0f,-6.0f,6.0f);  
  DBG("Init: peripherals started\r\n");
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc1);   
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t tim_dbg_tick = 0;
    if((now_tick - tim_dbg_tick) >= 1000){
    tim_dbg_tick = now_tick;

}

        motor_cli_poll();
        now_tick = HAL_GetTick();
      
        if((now_tick-led1_last_tick)>=500){
            led1_last_tick=now_tick;
            led1_state = !led1_state;
            if(led1_state)  LED1_ON;
            else            LED1_OFF;
        }
        if((now_tick-encoder_last_tick)>=5){
            encoder_last_tick=now_tick;
            if(as5600_read_mechanical_angle_rad(&motor2_encoder,&motor2_mechanical_angle_rad)==HAL_OK){
                motor2_electrical_angle_rad =foc_mechanical_to_electrical_angle(motor2_sensor_direction*motor2_mechanical_angle_rad,7,motor2_electrical_zero_offset_rad);
                motor2_angle_valid = 1;
                motor2_angle_last_ok_tick = now_tick;

               
            }
            else{
                if((now_tick - motor2_angle_last_ok_tick) > 50){
                        motor2_angle_valid = 0;
                        motor2_fault = 1;
                        DBG("ENCODER: timeout\r\n");
                }
            }
      
      
        }
        if(motor2_state != motor2_state_last){
                DBG("STATE -> %d\r\n", (int)motor2_state);
                motor2_state_last = motor2_state;
         }
        if (motor2_state == MOTOR2_STATE_CALIBRATING){
            if ((motor2_current.offset_calibrated != 0U) &&(motor2_angle_valid != 0)){
                    DBG("Calib done...");
                    motor2_state = MOTOR2_STATE_ALIGN;
                    DBG("Calib done: offset_calibrated=1, motor2_angle_valid=1\r\n");
            }       
        }
        
        if(motor2_state==MOTOR2_STATE_ALIGN){
            if((motor2_align_request !=0)&&(motor2_run_enable == 0)&&(motor2_align_started ==0)){
            
                motor2_align_request=0;
                DBG("ALIGN: request accepted\r\n");
                if (foc_svpwm(MOTOR2_ALIGN_VOLTAGE,0.0f,12.0f,&motor2_duty_u,&motor2_duty_v,&motor2_duty_w) == 0){
                      motor2_fault = 1;
                      DBG("ALIGN: svpwm fail\r\n");
                }
                else if (motor2_pwm_start() != HAL_OK){
                      motor2_fault = 1;
                      DBG("ALIGN: pwm start fail\r\n");
                }
                else{
                
                      motor2_pwm_set_duty(motor2_duty_u,motor2_duty_v,motor2_duty_w);
                      motor2_align_start_tick =now_tick;
                      motor2_align_started    =1; 
                      DBG("ALIGN: pwm started\r\n");

                }
                
            }
            if(motor2_align_started!=0){
                
                if((uint32_t)(now_tick - motor2_align_start_tick)>=500){
                    
                      if (as5600_read_mechanical_angle_rad(&motor2_encoder,&motor2_mechanical_angle_rad) == HAL_OK){
                            motor2_electrical_zero_offset_rad =foc_mechanical_to_electrical_angle(motor2_sensor_direction*motor2_mechanical_angle_rad,7,0.0f);
                            motor2_angle_valid = 1U;
                            DBG("ALIGN: done, offset=%.3f\r\n", motor2_electrical_zero_offset_rad);
                      }
                      else{               
                            motor2_fault = 1;
                            DBG("ALIGN: encoder read fail\r\n");
                      }
                      motor2_pwm_stop();
                      motor2_align_started = 0U;  
                      motor2_state=MOTOR2_STATE_READY;
            
                  }
            }
        
        
        }        
        /* OPENLOOP 进入条件 */
        if(motor2_state == MOTOR2_STATE_READY){
            if(motor2_openloop_enable != 0U){
                if(motor2_pwm_start() == HAL_OK){
                motor2_openloop_angle = 0.0f;   // 重置角度
                motor2_state = MOTOR2_STATE_OPENLOOP;
                DBG("OPENLOOP: started\r\n");
            }
            else{
                motor2_fault = 1;
            }
        }
        }
        if (motor2_state == MOTOR2_STATE_READY){
            if (motor2_run_enable != 0U){
                if (motor2_pwm_start() == HAL_OK){
                    motor2_state = MOTOR2_STATE_RUNNING;
                    DBG("RUN: pwm started\r\n");
                }
                else
                {
                    motor2_fault = 1;
                    DBG("RUN: pwm start fail\r\n");
                }
            }
        }
        if (motor2_state == MOTOR2_STATE_RUNNING)
        {
            if (motor2_run_enable == 0U)
            {
                motor2_pwm_stop();
                motor2_state = MOTOR2_STATE_READY;
                DBG("STOP: pwm stopped\r\n");
            }
        } 
        /* OPENLOOP 退出条件 */
        if(motor2_state == MOTOR2_STATE_OPENLOOP){
            if(motor2_openloop_enable == 0U){
                motor2_pwm_stop();
                motor2_state = MOTOR2_STATE_READY;
                DBG("OPENLOOP: stopped\r\n");
            }
        }
        if (motor2_fault != 0U)
        {
            motor2_run_enable = 0U;
            if (motor2_state != MOTOR2_STATE_FAULT)
            {
                motor2_pwm_stop();
                motor2_state = MOTOR2_STATE_FAULT;
                DBG("FAULT: entered\r\n");
            }
        }
        if(motor2_vofa.vofa_send_flag==1){
            static uint32_t vofa_send_cnt = 0;
            vofa_send(motor2_vofa.vofa_data,VOFA_DATA_NUM);
            motor2_vofa.vofa_send_flag=0;
            vofa_send_cnt++;
            if(vofa_send_cnt == 1){
            DBG("VOFA: first frame sent\r\n");
            }
        }
      
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */






void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
    volatile uint16_t adc_a_raw;
    volatile uint16_t adc_b_raw;
    static uint16_t vofa_count = 0;
    
    if(hadc->Instance==ADC1){
        adc_a_raw=(uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_1);
        adc_b_raw=(uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_2);
        if(!motor2_current.offset_calibrated){
              motor2_currentsense_calibration_sample(&motor2_current,adc_a_raw,adc_b_raw);
              return;
        }
        motor2_currentsense_update(&motor2_current,adc_a_raw,adc_b_raw);
        if(motor2_state == MOTOR2_STATE_OPENLOOP){
            /* 开环：自己生成电角度 */
            motor2_openloop_angle += motor2_openloop_speed * 0.00005f;  // 50us 周期
            if(motor2_openloop_angle > 6.2831853f){
            motor2_openloop_angle -= 6.2831853f;
            }

            /* 固定电压矢量：Vd=0, Vq=voltage */
                motor2_vd = 0.0f;
                motor2_vq = motor2_openloop_voltage;

            /* 反 Park */
            foc_inverse_park_transform(motor2_vd, motor2_vq,
                               motor2_openloop_angle,
                               &motor2_v_alpha, &motor2_v_beta);
        }
        else if(motor2_state == MOTOR2_STATE_RUNNING){
            /* 闭环*/
                foc_clarke_transform(motor2_current.current_a, motor2_current.current_b,
                         &motor2_i_alpha, &motor2_i_beta);
                foc_park_transform(motor2_i_alpha, motor2_i_beta,
                       motor2_electrical_angle_rad, &motor2_id, &motor2_iq);
                motor2_vd = foc_pi_update(&motor2_pi_d, motor2_id_ref - motor2_id, 0.00005f);
                motor2_vq = foc_pi_update(&motor2_pi_q, motor2_iq_ref - motor2_iq, 0.00005f);
                foc_inverse_park_transform(motor2_vd, motor2_vq,
                               motor2_electrical_angle_rad,
                               &motor2_v_alpha, &motor2_v_beta);
        }
        else{
                foc_pi_reset(&motor2_pi_d);
                foc_pi_reset(&motor2_pi_q);
                motor2_vd = 0.0f;
                motor2_vq = 0.0f;
                motor2_v_alpha = 0.0f;
                motor2_v_beta = 0.0f;
                return;
        }
        if(motor2_vofa.vofa_enable){
            vofa_count++;
            if(vofa_count>=100){
                vofa_count=0;
                if(motor2_vofa.vofa_send_flag==0){
                    vofa_capture(motor2_vofa.vofa_data);
                    motor2_vofa.vofa_send_flag=1;
                }
            }
        
        }
        
        if (foc_svpwm(motor2_v_alpha, motor2_v_beta,12.0f, &motor2_duty_u, &motor2_duty_v, &motor2_duty_w) == 0)
        {
            motor2_fault = 1;
            return;
        }
        if (((motor2_run_enable != 0U) || (motor2_openloop_enable != 0U)) 
            && (motor2_fault == 0U)){
            
                motor2_pwm_set_duty(motor2_duty_u, motor2_duty_v, motor2_duty_w);
            }


    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){


    if(huart->Instance == USART2){
        
        motor_cli_rx_char(uart2_rx_byte);
        HAL_UART_Receive_IT(&huart2,&uart2_rx_byte,1);
        
    }


}

















/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
