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
volatile uint32_t motor2_isr_cnt = 0;  
volatile uint32_t isr_cnt_last=0;

/* ---- 编码器/I2C 诊断：用来区分"真 I2C 故障"和"主循环被堵导致的假超时" ----
 * attempts/fails : 累计发起读取次数 / 失败次数
 * consec_fail    : 连续失败次数（判故障只看这个）
 * last_err       : 最近一次 HAL 返回码 1=HAL_ERROR(NACK) 2=HAL_BUSY 3=HAL_TIMEOUT
 * gap_try_max    : 两次"发起读取"之间的最大间隔(ms)，大 = 主循环被阻塞过
 * gap_ok_max     : 两次"成功读取"之间的最大间隔(ms)                              */
volatile uint32_t motor2_enc_attempts = 0;
volatile uint32_t motor2_enc_fails = 0;
volatile uint16_t motor2_enc_consec_fail = 0;
volatile int32_t  motor2_enc_last_err = 0;
volatile uint32_t motor2_enc_last_try_tick = 0;
volatile uint32_t motor2_enc_gap_try_max = 0;
volatile uint32_t motor2_enc_gap_ok_max = 0;
volatile uint32_t motor2_enc_recover_cnt = 0;   /* I2C 总线自愈执行次数 */

/* ---- 电角度外推：编码器 5ms 读一次，20kHz 的 ISR 必须用"当前时刻"的角度 ----
 * 问题：ISR 直接用电角度时最坏滞后 5ms，转子一转 δ=ω_e×5ms 就是几十度。
 *       实测（转子自由、IQ0.5）：iq 波动从锁轴时的 ~1.5 LSB 恶化到 31~51 LSB，
 *       频谱出现 200/400/600Hz 线谱（正好是 5ms 更新率的谐波）——一转动电流环就废。
 * 做法：主循环每次成功读到角度就记下 (机械角, 时刻)，并用两次读数差估算机械角速度
 *       （一阶低通抑制 12bit 编码器的量化噪声）；ISR 里
 *           电角度 = f(机械角_ref + ω·Δt)，Δt = 当前时刻 - 读数时刻
 * 时间基：TIM2 自由运行在 84MHz（main() 里 HAL_TIM_Base_Start），51s 回绕，
 *         Δt 用无符号相减，回绕天然正确。
 * 保护：主循环被 UART dump/CAP 阻塞时 Δt 会很大，外推钳位到 ANGLE_EXTRAP_MAX_S，
 *       并累计 extr_clip 计数（可在 STATUS 里看），避免角度被外推到离谱位置。 */
#define ANGLE_TICK_HZ        84000000.0f
#define ANGLE_EXTRAP_MAX_S   0.010f
#define ANGLE_EXTRAP_MAX_CNT ((uint32_t)(ANGLE_EXTRAP_MAX_S * ANGLE_TICK_HZ))
#define MECH_SPEED_LIMIT     300.0f               /* 机械角速度上限 rad/s（≈2865rpm） */
volatile float    motor2_mech_angle_ref = 0.0f;   /* 最近一次读到的机械角(带方向) */
volatile uint32_t motor2_angle_ref_cnt  = 0;      /* 该读数的 TIM2 计数 */
volatile float    motor2_mech_speed = 0.0f;       /* 滤波后的机械角速度 rad/s */
volatile uint8_t  motor2_mech_speed_valid = 0;    /* 0 = 还不足以估速（首次读数前） */
volatile uint32_t motor2_angle_extrap_clip = 0;   /* 诊断：外推被钳位次数 */
volatile uint32_t motor2_angle_extrap_max_us = 0; /* 诊断：最大外推时长(us) */
static float    mech_prev = 0.0f;
static uint32_t mech_prev_cnt = 0;
static uint8_t  mech_prev_valid = 0;
static uint16_t speed_div_cnt = 0;
static uint16_t caps_div_cnt = 0;

/* ---- 速度环（外环 PI）----
 * 节奏 5ms（每 SPEED_DIV 个 ISR 跑一次，与编码器更新同量级）；
 * 输入 motor2_mech_speed（编码器差分+低通），输出 iq_ref（限幅 ±SPEED_IQ_LIMIT）。
 * 使能：CLI `SPD <rad/s>`；手动 `IQ` 命令会自动退出速度环（人工优先）。
 * 注意 kp 的量纲是 A/(rad/s)，ki 是 A/(rad/s·s)；机械角速度，不是电角速度。 */
#define SPEED_DIV        100                     /* 20kHz/100 = 200Hz = 5ms */
#define SPEED_IQ_LIMIT   0.6f                    /* 速度环最多给多少 iq(A) */
foc_pi_t motor2_pi_s;
volatile float   motor2_speed_ref = 0.0f;        /* rad/s (机械) */
volatile uint8_t motor2_speed_enable = 0;
volatile float   motor2_iq_ref_speed = 0.0f;     /* 速度环输出（供观察） */

/* ---- 摩擦力前馈 ----
 * 为什么需要：低速时环路全靠积分慢慢攒电流，而静摩擦/齿槽是"台阶式"的，
 * 于是表现为"粘住 -> 攒够电流 -> 跳一格(还超调) -> 又粘住"的爬行。
 * 实测：本机静摩擦电流 ≈ 0.019 A（SPD 2 时速度 2s 都是 0，iq 才爬到 0.0185），
 *       SPD 5 时更是先静止 1s、然后一下冲到 7.7 rad/s (超调 54%)。
 *       位置环实测：0.03 比 0.02 好（1.5rad 步进的残差 -3.4° -> +1.7°）。
 * 做法：给定非零时按给定方向叠加一个固定前馈，电流一开始就跨过门槛，不用等积分。
 * 死区：|给定| 小于 SPEED_FF_DEADBAND 时不加，避免零速附近来回蹭。
 * 在线改：FF 0.03 / FF 0 */
#define SPEED_FF_DEFAULT   0.030f
#define SPEED_FF_DEADBAND  0.05f
#define SPEED_FF_RAMP      0.50f   /* 前馈在 |速度给定| 0.05->0.5 rad/s 之间线性升起：
                                    * 防止位置环在目标附近（速度给定很小）还拿满幅前馈来回顶，
                                    * 实测那会让转子以 ±5rad/s 在目标附近蹭。 */
volatile float   motor2_speed_ff = SPEED_FF_DEFAULT;

/* ---- 位置环（最外环）----
 * 结构：位置PI -> 速度给定 -> 速度环 -> iq_ref -> 电流环（三层级联，带宽 0.5Hz / 2.5Hz / 600Hz）。
 * 位置必须自己累圈：编码器是 12bit 绝对角(0~2pi)，只给单圈值，多圈要累加。
 *   在主循环每次成功读到角度时把"回绕修正后的增量"累加到 motor2_pos_mech。
 * 目标：POS x  表示"相对当前位置再转 x 弧度"（正方向与 SPD 正方向一致）。
 * 误差按 ±pi 回绕（走最短路径），跨圈时才不会绕远路。
 * 输出限幅 ±POS_SPD_LIMIT(rad/s)：位置误差再大，速度给定也不会爆掉。
 * 安全：编码器读数间隔过大（主循环被 UART dump 阻塞）时，圈数不可辨 —— 直接判位置失效
 *       并关掉位置环（POS_VALID=0），避免"多转一圈"这种意外动作。
 * 使能：POS 命令（自动打开速度环）；手动 IQ/SPD 命令会退出位置环（人工优先）。 */
#define POS_SPD_LIMIT   40.0f                    /* 位置环输出的速度给定上限 rad/s */
#define POS_GAP_MAX_S   0.030f                   /* 读数间隔超过它 -> 圈数不可辨 */
#define POS_DEADBAND    0.020f                   /* 位置死区(rad,≈±1.15°)：死区内停止追摩擦 */
foc_pi_t motor2_pi_pos;
volatile float   motor2_pos_mech  = 0.0f;        /* 累加机械角(rad, 多圈) */
volatile float   motor2_pos_ref   = 0.0f;        /* 目标(同类累加值) */
volatile float   motor2_pos_err   = 0.0f;        /* 最近一次位置误差 */
volatile uint8_t motor2_pos_enable = 0;
volatile uint8_t motor2_pos_valid  = 1;          /* 0 = 累圈不可信，位置环不能用 */
volatile uint32_t motor2_pos_gap_cnt = 0;        /* 诊断：因间隔过大而失效的次数 */
static uint16_t pos_div_cnt = 0;

/* ---- 速度/位置环抓取(CAPS)：200Hz × 400 点 = 2s ----
 * 内容 pos_mech, pos_ref, speed_ref, mech_speed, iq_ref, iq
 * （电流环的 CAP 只有 20ms，看不了 5ms 的速度环/位置环） */
#define CAPS_N   400
#define CAPS_DIV 100
volatile uint8_t  caps_armed = 0;
volatile uint16_t caps_n = 0;
float caps_buf[CAPS_N][6];

/* 对齐用暂存量：等转子被磁场拉到位并真正停下来，再锁存零位（见 ALIGN 分支注释） */
float    motor2_align_last_mech = 0.0f;
uint8_t  motor2_align_stable_cnt = 0;
uint32_t motor2_align_read_tick = 0;

/* ---- ISR 内 RAM 抓取（看电流环动态/阶跃响应）----
 * 用法：先发 CAP 命令 arm；随后"下一次 ID/IQ 给定变化"开始按 ISR 节奏(50us)记录 CAP_N 点，
 *       填满后主循环把数据以 ASCII 行打入串口（CAP BEGIN / CAP END 包裹，自带标签，无解析歧义）。
 * 抓取内容：id_ref, iq_ref, id, iq, vd, vq —— 20ms 窗口足够看清 1kHz 环路的上升/超调/振荡。
 * 注意：dump 会阻塞主循环约 1.7s（期间编码器不更新），所以抓取时转子应锁住/静止。 */
#define CAP_N 400
volatile uint8_t  cap_armed = 0;      /* CAP 命令置 1，等给定变化触发 */
volatile uint8_t  cap_filling = 0;    /* 正在记录 */
volatile uint16_t cap_n = 0;          /* 已记录点数（>=CAP_N 表示待 dump） */
float cap_buf[CAP_N][6];

float motor2_electrical_zero_offset_rad = 0.0f;
int8_t motor2_sensor_direction = 1;

float motor2_mechanical_angle_rad;
volatile float motor2_electrical_angle_rad;
uint32_t encoder_last_tick;
uint32_t now_tick;
static uint32_t led1_last_tick = 0;
static uint32_t motor2_angle_last_ok_tick = 0;
static uint32_t motor2_align_start_tick = 0U;   
static uint8_t  motor2_align_started    = 0U;   

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
volatile float motor2_openloop_speed = 5.0f;      // 电角度速度 rad/s
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

  /* TIM2 自由运行当 84MHz 时间戳（电角度外推要用，见文件上方的说明） */
  HAL_TIM_Base_Start(&htim2);

  encoder_last_tick = HAL_GetTick();
  motor2_current_init(&motor2_current,0,0);
  motor2_currentsense_calibration_start(&motor2_current);

  /* 电流环 PI 上电默认值（d/q 用同一套：受控对象相同）。
   * kp = 2, ki = 10000 为实测标定值（20kHz 电流环，dt = 50us）：
   *   - ki/kp = 0.2ms，正好等于 L/R = 0.68mH / 3.4ohm，做零极点对消；
   *   - 实测：稳态误差 ~1%，上升 0.25~0.30ms，kp=2 时超调约 25%；
   *     锁轴验证 vq = 0.34V @ iq = 0.1A 对应 R = 3.4ohm，量纲自洽。
   * 也可以在运行时用 CLI 改：PID_D kp ki / PID_Q kp ki（掉电丢失）。
   * 输出限幅 ±6V；实际可用矢量电压由 foc_svpwm 的 span 限幅再压到约 4V。 */
  foc_pi_init(&motor2_pi_d,2.0f,10000.0f,-6.0f,6.0f);
  foc_pi_init(&motor2_pi_q,2.0f,10000.0f,-6.0f,6.0f);  
  /* 速度环 PI 上电默认值：实测扫出来的（CAPS 抓 2s 阶跃）
   *   kp=0.001 ki=0.005 -> 上升 ~400ms、超调 4%、稳态误差 -0.2%、波动 ±1%（给定 30rad/s）
   * 注意量级：本机机械阻尼很小，kp 给到 0.005 就开始振，ki 给到 0.1 会出现极限环
   * （纯惯量对象上积分=双积分器，相位天生 180°），所以别凭直觉往上加。
   * 在线改：PID_S 0.001 0.005 */
  foc_pi_init(&motor2_pi_s,0.001f,0.005f,-SPEED_IQ_LIMIT,SPEED_IQ_LIMIT);
  /* 位置环 PI 上电默认值：kp 量纲 1/s（输出=速度给定=kp×位置误差），ki 是 1/s²。
   * 目标带宽 ~0.5Hz，必须远低于速度环的 ~2.5Hz —— 三层级联要保证带宽阶梯。 */
  foc_pi_init(&motor2_pi_pos,3.0f,1.0f,-POS_SPD_LIMIT,POS_SPD_LIMIT);
  DBG("Init: peripherals started\r\n");
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


        /* 【已停用】这段 500ms 方波测试给定会覆盖 CLI 的 ID 命令（主循环每轮都写 id_ref），
         * 导致用 `ID x` 做稳态/阶跃测试时 id_ref 实际在 0/0.2 之间跳变，测出来没有意义。
         * 需要方波给定做阶跃测试时，再放开它，或用命令面板连续发 ID 0.2 / ID 0。 */
#if 0
        if (((HAL_GetTick() / 500U) & 1U) != 0U)
        {
            motor2_id_ref = 0.2f;
        }
        else
        {
            motor2_id_ref = 0.0f;
        }
#endif


        static uint32_t tim_dbg_tick = 0;
//        static uint32_t enc_fails_last = 0;   /* 下面诊断块打开时一起放开 */
        if((now_tick - tim_dbg_tick) >= 1000){
            tim_dbg_tick = now_tick;
//            /* 诊断怎么看：
//             *   gapTry 大(>500ms)  → 主循环被 UART 发送阻塞过：那段时间根本没发起读取，
//             *                        这种情况属于"假超时"，与 I2C 硬件无关
//             *   fails 持续增长且 st!=0 → I2C 真的在读失败：
//             *                        st=1 NACK(从机没应答/干扰)  2 BUSY  3 TIMEOUT(总线被拉死)
//             *   gapOk 大但没有 fail → 读数一直是成功的，只是间隔被拉长
//             * scl/sda = PB6/PB7 引脚当前实际电平(AF 模式也能读 IDR)：
//             *   两条都=1 → 总线其实是空闲的，那 HAL_BUSY 是外设/句柄状态卡住；
//             *   有一条=0 → 总线被物理拉住（从机没松手 / 上拉不足 / 短路）       */
//            {
//                uint8_t scl_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;
//                uint8_t sda_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
//                DBG("isr=%lu/s enc try=%lu fails=%lu(+%lu) consec=%u st=%ld rc=%lu scl=%u sda=%u gapTry=%lums gapOk=%lums\r\n",
//                    (unsigned long)(motor2_isr_cnt - isr_cnt_last),
//                    (unsigned long)motor2_enc_attempts,
//                    (unsigned long)motor2_enc_fails,
//                    (unsigned long)(motor2_enc_fails - enc_fails_last),
//                    (unsigned)motor2_enc_consec_fail,
//                    (long)motor2_enc_last_err,
//                    (unsigned long)motor2_enc_recover_cnt,
//                    (unsigned)scl_lvl,
//                    (unsigned)sda_lvl,
//                    (unsigned long)motor2_enc_gap_try_max,
//                    (unsigned long)motor2_enc_gap_ok_max);
//            }
//            enc_fails_last = motor2_enc_fails;
//            motor2_enc_gap_try_max = 0;
//            motor2_enc_gap_ok_max = 0;              /* ← 新增 */
//            isr_cnt_last = motor2_isr_cnt;   
        }

        /* RAM 抓取数据回传（ASCII 带标签，避免二进制解析歧义） */
        if (cap_n >= CAP_N) {
            uint16_t k;
            DBG("CAP BEGIN n=%u\r\n", (unsigned)CAP_N);
            for (k = 0; k < CAP_N; k++) {
                DBG("C %u %.4f %.4f %.4f %.4f %.4f %.4f\r\n", (unsigned)k,
                    cap_buf[k][0], cap_buf[k][1], cap_buf[k][2],
                    cap_buf[k][3], cap_buf[k][4], cap_buf[k][5]);
            }
            DBG("CAP END\r\n");
            cap_n = 0;
        }

        /* 速度环/位置环抓取回传：S idx pos_mech pos_ref speed_ref speed iq_ref iq */
        if (caps_n >= CAPS_N) {
            uint16_t k;
            DBG("CAPS BEGIN n=%u\r\n", (unsigned)CAPS_N);
            for (k = 0; k < CAPS_N; k++) {
                DBG("S %u %.4f %.4f %.4f %.4f %.4f %.4f\r\n", (unsigned)k,
                    caps_buf[k][0], caps_buf[k][1], caps_buf[k][2],
                    caps_buf[k][3], caps_buf[k][4], caps_buf[k][5]);
            }
            DBG("CAPS END\r\n");
            caps_n = 0;
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
            HAL_StatusTypeDef enc_st;
            uint32_t enc_gap;

            encoder_last_tick=now_tick;

            /* 记录"发起读取"的间隔：主循环被 UART 阻塞时不发起读取，这里会变大 */
            motor2_enc_attempts++;
            enc_gap = now_tick - motor2_enc_last_try_tick;
            if(enc_gap > motor2_enc_gap_try_max){ motor2_enc_gap_try_max = enc_gap; }
            motor2_enc_last_try_tick = now_tick;

            enc_st = as5600_read_mechanical_angle_rad(&motor2_encoder,&motor2_mechanical_angle_rad);
            if(enc_st==HAL_OK){
                /* ---- 机械角/时刻/角速度：供 ISR 做电角度外推（见文件上方说明）---- */
                float    mech     = motor2_sensor_direction * motor2_mechanical_angle_rad;
                uint32_t now_cnt  = TIM2->CNT;

                if(mech_prev_valid){
                    /* 编码器是 12bit 绝对角(0~2pi)，先做回绕修正 */
                    float d = mech - mech_prev;
                    if(d >  3.14159265f){ d -= 6.28318531f; }
                    if(d < -3.14159265f){ d += 6.28318531f; }
                    /* 多圈位置累加（位置环用）。前提是两次读数间隔够短，
                     * 否则"转过几圈"不可辨 —— 见 dt 判断 */
                    motor2_pos_mech += d;
                    {
                        float dt = (float)(uint32_t)(now_cnt - mech_prev_cnt) / ANGLE_TICK_HZ;
                        /* dt 太大 = 主循环刚被 UART dump 阻塞过，这段差不可信，丢弃 */
                        if((dt > 0.0002f) && (dt < 0.2f)){
                            float w = d / dt;
                            /* 一阶低通：编码器 1LSB=0.088°机械，5ms 差分噪声约 ±0.6rad/s，
                             * 系数 0.25 ≈ 20ms 时间常数；外推只有 5ms，噪声放大有限 */
                            motor2_mech_speed += 0.25f * (w - motor2_mech_speed);
                            /* 编码器偶发跳数（或回绕算错）会给出离谱的速度，
                             * 钳一下避免外推角度被甩飞：300rad/s ≈ 2865rpm，远超本机范围 */
                            if(motor2_mech_speed >  MECH_SPEED_LIMIT){ motor2_mech_speed =  MECH_SPEED_LIMIT; }
                            if(motor2_mech_speed < -MECH_SPEED_LIMIT){ motor2_mech_speed = -MECH_SPEED_LIMIT; }
                            motor2_mech_speed_valid = 1U;
                        }
                        else if(dt >= POS_GAP_MAX_S){
                            /* 间隔过大：这一段的圈数不可辨，位置失效，
                             * 并关掉位置环（否则可能驱动到"差一圈"的错误目标） */
                            if(motor2_pos_valid){
                                motor2_pos_gap_cnt++;
                                motor2_pos_valid  = 0U;
                                motor2_pos_enable = 0U;
                            }
                        }
                    }
                }
                mech_prev       = mech;
                mech_prev_cnt   = now_cnt;
                mech_prev_valid = 1U;
                motor2_mech_angle_ref = mech;
                motor2_angle_ref_cnt  = now_cnt;

                motor2_electrical_angle_rad =foc_mechanical_to_electrical_angle(mech,7,motor2_electrical_zero_offset_rad);
                motor2_angle_valid = 1;
                motor2_enc_consec_fail = 0;
                enc_gap = now_tick - motor2_angle_last_ok_tick;
                if(enc_gap > motor2_enc_gap_ok_max){ motor2_enc_gap_ok_max = enc_gap; }
                motor2_angle_last_ok_tick = now_tick;
            }
            else{
                motor2_enc_fails++;
                motor2_enc_last_err = (int32_t)enc_st;
                if(motor2_enc_consec_fail < 0xFFFF){ motor2_enc_consec_fail++; }
                /* 只在"连败开始"时打印一次，避免真故障时刷屏把主循环彻底堵死 */
                if(motor2_enc_consec_fail == 1){
                    DBG("ENC fail st=%ld gap=%lums\r\n",
                        (long)enc_st,
                        (unsigned long)(now_tick - motor2_angle_last_ok_tick));
                }
                /* 连续失败到第 6 次：先执行一次 I2C 总线自愈。
                 * st=2(HAL_BUSY) 就是总线被拉住（从机停在半个字节里），
                 * 不打 9 个时钟永远好不了；自愈成功则下次读取成功、consec_fail 自动清零 */
                if(motor2_enc_consec_fail == 6){
                    motor2_enc_recover_cnt++;
                    DBG("ENC: 6 consec fails (st=%ld) -> I2C bus recover #%lu\r\n",
                        (long)enc_st, (unsigned long)motor2_enc_recover_cnt);
                    if(as5600_bus_recover(&motor2_encoder) == 0){
                        DBG("  -> SCL held low by AS5600/hardware: software cannot fix, power-cycle the AS5600\r\n");
                    }else{
                        DBG("  -> SDA stuck: clocked 9 bits\r\n");
                    }
                }
                /* 自愈后仍连续失败到 20 次（≈100ms）才判故障：确实坏透了才停机。
                 * 只在第 20 次打印一次（fault 一旦置位会保持，不必重复设定） */
                if(motor2_enc_consec_fail == 20){
                        motor2_angle_valid = 0;
                        motor2_fault = 1;
                        DBG("ENCODER: fault (20 consec fails, st=%ld)\r\n", (long)enc_st);
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
                      motor2_align_stable_cnt = 0;      /* 复位对齐稳定性判断 */
                      motor2_align_last_mech  = 0.0f;
                      motor2_align_read_tick  = 0;
                      DBG("ALIGN: pwm started\r\n");

                }

            }
            if(motor2_align_started!=0){

                /* 不能固定 500ms 读一次就当零位：转子被磁场拉到位后会在 ~19Hz 上欠阻尼振荡，
                 * 振荡中途读数会让零位偏几十度 -> 之后"纯 d 轴电流"还会推动转子。
                 * 做法：500ms 之后每 50ms 读一次，连续 3 次(≈150ms)几乎不动才认为稳定并锁存；
                 *       最长等 3s 兜底。 */
                if(((uint32_t)(now_tick - motor2_align_start_tick)>=500) &&
                   ((uint32_t)(now_tick - motor2_align_read_tick) >= 50)){

                      motor2_align_read_tick = now_tick;
                      if (as5600_read_mechanical_angle_rad(&motor2_encoder,&motor2_mechanical_angle_rad) == HAL_OK){
                            float dmech = motor2_mechanical_angle_rad - motor2_align_last_mech;
                            if (dmech < 0.0f) { dmech = -dmech; }
                            if (dmech < 0.005f) { motor2_align_stable_cnt++; }
                            else                { motor2_align_stable_cnt = 0; }
                            motor2_align_last_mech = motor2_mechanical_angle_rad;

                            motor2_electrical_zero_offset_rad =foc_mechanical_to_electrical_angle(motor2_sensor_direction*motor2_mechanical_angle_rad,7,0.0f);
                            motor2_angle_valid = 1U;

                            if((motor2_align_stable_cnt >= 3) ||
                               ((uint32_t)(now_tick - motor2_align_start_tick) >= 3000)){
                                  DBG("ALIGN: done, offset=%.3f (t=%lums)\r\n",
                                      motor2_electrical_zero_offset_rad,
                                      (unsigned long)(now_tick - motor2_align_start_tick));
                                  motor2_pwm_stop();
                                  motor2_align_started = 0U;
                                  motor2_align_stable_cnt = 0;
                                  motor2_state=MOTOR2_STATE_READY;
                            }
                      }
                      else{               
                            motor2_fault = 1;
                            DBG("ALIGN: encoder read fail\r\n");
                            motor2_pwm_stop();
                            motor2_align_started = 0U;
                            motor2_state = MOTOR2_STATE_FAULT;
                      }

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






/* FAULT 清除：回到 READY，等新的 ALIGN / OPENLOOP / RUN 命令。
 * 一并清掉 openloop_enable（否则状态机一回 READY 就会自动重进开环），
 * 并复位编码器连败计数，让下一次运行从干净状态开始。 */
void motor2_fault_clear(void)
{
    motor2_fault = 0;
    motor2_run_enable = 0;
    motor2_openloop_enable = 0;
    /* 外环也使能一并清掉：否则清完故障再 RUN，速度环/位置环会直接朝旧给定冲过去 */
    motor2_speed_enable = 0;
    motor2_pos_enable   = 0;
    motor2_iq_ref       = 0.0f;
    motor2_align_request = 0;
    motor2_align_started = 0;
    motor2_enc_consec_fail = 0;
    motor2_state = MOTOR2_STATE_READY;
    DBG("FAULT cleared -> READY\r\n");
}

/* 给 CLI 用：状态机当前状态号
 * 0=CALIBRATING 1=ALIGN 2=READY 3=RUNNING 4=OPENLOOP 5=FAULT */
int motor2_state_get(void)
{
    return (int)motor2_state;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
    volatile uint16_t adc_a_raw;
    volatile uint16_t adc_b_raw;
    static uint16_t vofa_count = 0;

     motor2_isr_cnt++;       
    
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
            /* ---- ① 电角度外推到当前时刻（编码器 5ms 才读一次，这里是 50us 一拍）----
             * 这是"转子一转电流环就废"的根治：之前 ISR 直接用主循环写下的角度，
             * 最坏滞后 5ms，ω_e=100rad/s 时就是 28° 的坐标系偏差。 */
            {
                uint32_t dcnt = (uint32_t)(TIM2->CNT - motor2_angle_ref_cnt);
                if(dcnt > ANGLE_EXTRAP_MAX_CNT){
                    dcnt = ANGLE_EXTRAP_MAX_CNT;            /* 主循环被 dump 阻塞时的保护 */
                    motor2_angle_extrap_clip++;
                }
                if(dcnt > motor2_angle_extrap_max_us * 84U){
                    motor2_angle_extrap_max_us = dcnt / 84U;
                }
                if(motor2_mech_speed_valid){
                    float mech = motor2_mech_angle_ref
                               + motor2_mech_speed * ((float)dcnt / ANGLE_TICK_HZ);
                    motor2_electrical_angle_rad =
                        foc_mechanical_to_electrical_angle(mech,7,motor2_electrical_zero_offset_rad);
                }
            }

            /* ---- ② 位置环（最外环，与速度环同一节拍，输出直接改速度给定）---- */
            if(motor2_pos_enable){
                pos_div_cnt++;
                if(pos_div_cnt >= SPEED_DIV){
                    float pos_err;
                    pos_div_cnt = 0;
                    if(!motor2_pos_valid){
                        motor2_pos_enable = 0;      /* 累圈不可信：退出位置环，避免跑错整圈 */
                        motor2_iq_ref = 0.0f;
                    }
                    else{
                        pos_err = motor2_pos_ref - motor2_pos_mech;
                        /* 按 ±pi 回绕：跨圈时走最短路径 */
                        while(pos_err >  3.14159265f){ pos_err -= 6.28318531f; }
                        while(pos_err < -3.14159265f){ pos_err += 6.28318531f; }
                        motor2_pos_err = pos_err;
                        if((pos_err < POS_DEADBAND) && (pos_err > -POS_DEADBAND)){
                            /* 死区（≈±1.15°）：不再和静摩擦较劲。
                             * 输出 0 速度给定，并把内环(速度环)积分也清掉 ——
                             * 否则速度环积分会慢慢把电流顶到摩擦门槛，又蹭起来。 */
                            motor2_speed_ref = 0.0f;
                            motor2_pi_pos.integral = 0.0f;
                            foc_pi_reset(&motor2_pi_s);
                        }
                        else{
                            motor2_speed_ref = foc_pi_update(&motor2_pi_pos, pos_err,
                                                    (float)SPEED_DIV / 20000.0f);
                        }
                        motor2_speed_enable = 1;    /* 位置环要工作，内环(速度环)必须在环 */
                    }
                }
            }
            else{
                pos_div_cnt = 0;
            }

            /* ---- ③ 速度环（外环 PI，5ms 一拍）----
             * 用外推后的电角度做电流环、用机械角速度做速度环，两者同一套角度来源。 */
            if(motor2_speed_enable){
                speed_div_cnt++;
                if(speed_div_cnt >= SPEED_DIV){
                    float iq_cmd, iq_ff;
                    speed_div_cnt = 0;
                    iq_cmd = foc_pi_update(&motor2_pi_s,
                                            motor2_speed_ref - motor2_mech_speed,
                                            (float)SPEED_DIV / 20000.0f);
                    /* 摩擦力前馈：按给定方向叠加，但要随 |速度给定| 逐渐升起。
                     * 满幅前馈 = 0.03A 比静摩擦门槛(0.019A)还大，如果在目标附近一直满幅，
                     * 就会把转子顶得来回蹭（实测 SPD=-5rad/s 摇摆、位置 ±2° 抖）。 */
                    iq_ff = 0.0f;
                    {
                        float s   = motor2_speed_ref;
                        float mag = (s >= 0.0f) ? s : -s;
                        if(mag > SPEED_FF_DEADBAND){
                            float g = (mag - SPEED_FF_DEADBAND) /
                                      (SPEED_FF_RAMP - SPEED_FF_DEADBAND);
                            if(g > 1.0f){ g = 1.0f; }
                            iq_ff = (s >= 0.0f) ? (motor2_speed_ff * g)
                                                : (-motor2_speed_ff * g);
                        }
                    }
                    iq_cmd += iq_ff;
                    if(iq_cmd >  SPEED_IQ_LIMIT){ iq_cmd =  SPEED_IQ_LIMIT; }
                    if(iq_cmd < -SPEED_IQ_LIMIT){ iq_cmd = -SPEED_IQ_LIMIT; }
                    motor2_iq_ref_speed = iq_cmd;
                    motor2_iq_ref = iq_cmd;
                }
            }
            else{
                speed_div_cnt = 0;
            }

            /* 闭环 */
                /* 用滤波后的电流做 Clarke/Park：原始读数分辨率只有 3.22mA/LSB
                 * 且带 3~6 LSB 抖动，直接进 PI 会被放大成真实的电流摆动（详见 current.h） */
                foc_clarke_transform(motor2_current.current_a_f, motor2_current.current_b_f,
                         &motor2_i_alpha, &motor2_i_beta);
                foc_park_transform(motor2_i_alpha, motor2_i_beta,
                       motor2_electrical_angle_rad, &motor2_id, &motor2_iq);
                motor2_vd = foc_pi_update(&motor2_pi_d, motor2_id_ref - motor2_id, 0.00005f);
                motor2_vq = foc_pi_update(&motor2_pi_q, motor2_iq_ref - motor2_iq, 0.00005f);
                foc_inverse_park_transform(motor2_vd, motor2_vq,
                               motor2_electrical_angle_rad,
                               &motor2_v_alpha, &motor2_v_beta);
                /* RAM 抓取：CAP 之后的第一次给定变化开始记录（每 50us 一点） */
                if (cap_filling && (cap_n < CAP_N)) {
                    cap_buf[cap_n][0] = motor2_id_ref;
                    cap_buf[cap_n][1] = motor2_iq_ref;
                    cap_buf[cap_n][2] = motor2_id;
                    cap_buf[cap_n][3] = motor2_iq;
                    cap_buf[cap_n][4] = motor2_vd;
                    cap_buf[cap_n][5] = motor2_vq;
                    cap_n++;
                    if (cap_n >= CAP_N) { cap_filling = 0; }
                }
                /* 速度环抓取（CAPS）：200Hz 采样，2s 窗口，看速度阶跃/跟踪 */
                if (caps_armed) {
                    caps_div_cnt++;
                    if (caps_div_cnt >= CAPS_DIV) {
                        caps_div_cnt = 0;
                        if (caps_n < CAPS_N) {
                            caps_buf[caps_n][0] = motor2_pos_mech;
                            caps_buf[caps_n][1] = motor2_pos_ref;
                            caps_buf[caps_n][2] = motor2_speed_ref;
                            caps_buf[caps_n][3] = motor2_mech_speed;
                            caps_buf[caps_n][4] = motor2_iq_ref;
                            caps_buf[caps_n][5] = motor2_iq;
                            caps_n++;
                            if (caps_n >= CAPS_N) { caps_armed = 0; }
                        }
                    }
                }
        }
        else{
                foc_pi_reset(&motor2_pi_d);
                foc_pi_reset(&motor2_pi_q);
                foc_pi_reset(&motor2_pi_s);
                foc_pi_reset(&motor2_pi_pos);
                speed_div_cnt = 0;
                pos_div_cnt   = 0;
                caps_div_cnt  = 0;
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
