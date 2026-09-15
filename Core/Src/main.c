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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MOTOR1_ALIGN_VOLTAGE 0.5f

typedef enum
{
    MOTOR1_STATE_CALIBRATING = 0,
    MOTOR1_STATE_ALIGN,
    MOTOR1_STATE_READY,
    MOTOR1_STATE_RUNNING,
    MOTOR1_STATE_FAULT

} motor1_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

motor1_currentsense_t motor1_current;
as5600_t motor1_encoder;

float motor1_electrical_zero_offset_rad = 0.0f;
int8_t motor1_sensor_direction = 1;

float motor1_mechanical_angle_rad;
volatile float motor1_electrical_angle_rad;
uint32_t encoder_last_tick;
uint32_t now_tick;
static uint32_t motor1_align_start_tick = 0U;   /* 对齐开始时刻 */
static uint8_t  motor1_align_started    = 0U;   /* 是否已启动对齐 */

foc_pi_t motor1_pi_d;
foc_pi_t motor1_pi_q;

volatile  float motor1_id_ref=0.0f;
volatile  float motor1_iq_ref=0.0f;

float motor1_i_alpha;
float motor1_i_beta;
float motor1_id;
float motor1_iq;

float motor1_vd;
float motor1_vq;
float motor1_v_alpha;
float motor1_v_beta;

float motor1_duty_u;
float motor1_duty_v;
float motor1_duty_w;

volatile uint8_t motor1_fault;
volatile uint8_t motor1_run_enable = 0;

volatile motor1_state_t motor1_state =MOTOR1_STATE_CALIBRATING;
volatile uint8_t motor1_angle_valid = 0;

uint8_t uart2_rx_byte;
volatile uint8_t motor1_align_request=0;

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2,&uart2_rx_byte,1);
  as5600_init(&motor1_encoder, &hi2c1);
  encoder_last_tick = HAL_GetTick();
  motor1_current_init(&motor1_current,0,0);
  motor1_currentsense_calibration_start(&motor1_current);
  
  foc_pi_init(&motor1_pi_d,0.0f,0.0f,-6.0f,6.0f);
  foc_pi_init(&motor1_pi_q,0.0f,0.0f,-6.0f,6.0f);  
  
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      
        now_tick = HAL_GetTick();
        if((now_tick-encoder_last_tick)>=5){
            encoder_last_tick=now_tick;
            if(as5600_read_mechanical_angle_rad(&motor1_encoder,&motor1_mechanical_angle_rad)==HAL_OK){
          
                motor1_electrical_angle_rad =foc_mechanical_to_electrical_angle(motor1_sensor_direction*motor1_mechanical_angle_rad,7,motor1_electrical_zero_offset_rad);
                motor1_angle_valid = 1;
            }
      
      
        }
        if (motor1_state == MOTOR1_STATE_CALIBRATING){
            if ((motor1_current.offset_calibrated != 0U) &&(motor1_angle_valid != 0)){
                    motor1_state = MOTOR1_STATE_ALIGN;
            }       
        }
        
        if(motor1_state==MOTOR1_STATE_ALIGN){
            if((motor1_align_request !=0)&&(motor1_run_enable == 0)&&(motor1_align_started ==0)){
            
                motor1_align_request=0;
                
                if (foc_svpwm(MOTOR1_ALIGN_VOLTAGE,0.0f,12.0f,&motor1_duty_u,&motor1_duty_v,&motor1_duty_w) == 0){
                      motor1_fault = 1;
                }
                else if (motor1_pwm_start() != HAL_OK){
                      motor1_fault = 1;
                }
                else{
                
                      motor1_pwm_set_duty(motor1_duty_u,motor1_duty_v,motor1_duty_w);
                      motor1_align_start_tick =now_tick;
                      motor1_align_started    =1; 

                }
                
            }
            if(motor1_align_started!=0){
                
                if((uint32_t)(now_tick - motor1_align_start_tick)>=500){
                    
                      if (as5600_read_mechanical_angle_rad(&motor1_encoder,&motor1_mechanical_angle_rad) == HAL_OK){
                            motor1_electrical_zero_offset_rad =foc_mechanical_to_electrical_angle(motor1_sensor_direction*motor1_mechanical_angle_rad,7,0.0f);
                            motor1_angle_valid = 1U;
                      }
                      else{               
                            motor1_fault = 1;
                      }
                      motor1_pwm_stop();
                      motor1_align_started = 0U;  
                      motor1_state=MOTOR1_STATE_READY;
            
                  }
            }
        
        
        }        
        
        if (motor1_state == MOTOR1_STATE_READY){
            if (motor1_run_enable != 0U){
                if (motor1_pwm_start() == HAL_OK){
                    motor1_state = MOTOR1_STATE_RUNNING;
                }
                else
                {
                    motor1_fault = 1;
                }
            }
        }
        if (motor1_state == MOTOR1_STATE_RUNNING)
        {
            if (motor1_run_enable == 0U)
            {
                motor1_pwm_stop();
                motor1_state = MOTOR1_STATE_READY;
            }
        } 
        if (motor1_fault != 0U)
        {
            motor1_run_enable = 0U;
            if (motor1_state != MOTOR1_STATE_FAULT)
            {
                motor1_pwm_stop();
                motor1_state = MOTOR1_STATE_FAULT;
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
    
    if(hadc->Instance==ADC1){
        adc_a_raw=(uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_1);
        adc_b_raw=(uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_2);
        if(!motor1_current.offset_calibrated){
              motor1_currentsense_calibration_sample(&motor1_current,adc_a_raw,adc_b_raw);
              return;
        }
        motor1_currentsense_update(&motor1_current,adc_a_raw,adc_b_raw);
        if(motor1_state != MOTOR1_STATE_RUNNING){
            foc_pi_reset(&motor1_pi_d);
            foc_pi_reset(&motor1_pi_q);
            motor1_vd = 0.0f;
            motor1_vq = 0.0f;
            motor1_v_alpha = 0.0f;
            motor1_v_beta = 0.0f;
            return;
        }
        foc_clarke_transform(motor1_current.current_a,motor1_current.current_b,&motor1_i_alpha,&motor1_i_beta);
        foc_park_transform(motor1_i_alpha,motor1_i_beta,motor1_electrical_angle_rad,&motor1_id,&motor1_iq);
        motor1_vd =foc_pi_update(&motor1_pi_d,motor1_id_ref-motor1_id,0.00005f);
        motor1_vq =foc_pi_update(&motor1_pi_q,motor1_iq_ref-motor1_iq,0.00005f);    
        foc_inverse_park_transform (motor1_vd,motor1_vq,motor1_electrical_angle_rad,&motor1_v_alpha,&motor1_v_beta);       
        if (foc_svpwm(motor1_v_alpha, motor1_v_beta,12.0f, &motor1_duty_u, &motor1_duty_v, &motor1_duty_w) == 0)
        {
            motor1_fault = 1;
            return;
        }
        if ((motor1_run_enable != 0U) &&(motor1_fault == 0U)){
            
            motor1_pwm_set_duty(motor1_duty_u,motor1_duty_v,motor1_duty_w);
        
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
