# STM32F401 双电机 FOC 框架需求与架构设计

## 1. 文档目的

本文档定义 `stm32_foc` 工程的第一版双电机 FOC 框架需求、硬件边界、软件架构、控制时序、故障策略和验收标准。目标是形成一套可在当前 STM32F401RCT6 工程中逐步实现、可测量验证、可继续扩展的控制框架。

本文档只定义控制器需求和接口，不修改功率驱动芯片、电流采样电路或电机机械结构。

## 2. 已确认的系统基线

### 2.1 MCU 与工程

- MCU：STM32F401RCT6
- CPU 时钟：84 MHz
- 工程方式：STM32CubeMX 生成的 HAL 工程，Keil MDK-ARM
- 工程入口：`Core/Src/main.c`
- 外设配置源：`stm32_foc.ioc`
- 不引入 Arduino、ESP-IDF 或 RTOS
- 快速控制环使用定时器/ADC 中断，慢速任务使用主循环或定时标志

### 2.2 双电机 PWM

| 电机 | 定时器 | 输出通道 | 引脚 | 模式 |
|---|---|---|---|---|
| Motor 1 | TIM1 | CH1/CH2/CH3 | PA8/PA9/PA10 | 中心对齐，ARR=2100-1，约 20 kHz |
| Motor 2 | TIM3 | CH1/CH2/CH3 | PA6/PA7/PB0 | 中心对齐，ARR=2100-1，约 20 kHz |

PWM 频率计算：

```text
84 MHz / (2 × 2100) = 20 kHz
```

控制器只输出每个电机 3 路 PWM。驱动芯片负责互补驱动、死区和功率级过流保护，MCU 不生成 6PWM，不使用 TIM1/TIM3 Break 作为本版本的保护入口。

### 2.3 电流采样

- 采样拓扑：低端分流电阻
- 每路分流电阻：5 mΩ
- 电流检测器件：INA240A2PW
- 放大倍数：50 V/V
- 电流采样灵敏度：`0.005 Ω × 50 = 0.25 V/A`
- 每个电机采集两相电流，第三相使用相电流和为零重构

当前 ADC 资源基线：

| 电机 | ADC 组 | 当前通道 | 触发/搬运方式 |
|---|---|---|---|
| Motor 1 | ADC1 注入组 | PA4/ADC1_IN4、PA5/ADC1_IN5 | TIM1_CH4 触发 |
| Motor 2 | ADC1 常规组 | PB1/ADC1_IN9、PC5/ADC1_IN15 | TIM3_TRGO 触发，DMA 搬运 |

STM32F401 只有 ADC1，因此必须增加 ADC 共享资源管理，保证两个采样序列不重叠、不丢失、不误归属。

### 2.4 母线与位置反馈

- 母线额定电压：12 V
- 母线电压未接入 MCU ADC
- 母线过压、欠压和电源异常由外部电源/驱动芯片处理
- 位置传感器：每个电机一个 AS5600
- 本框架采用 I²C 读取 AS5600 角度
- Motor 1 使用 I2C1，Motor 2 使用 I2C2
- AS5600 默认 7 位地址：`0x36`
- AS5600 角度为 12 bit，读取 `ANGLE` 寄存器 `0x0E/0x0F`

### 2.5 电机参数基线

两个电机第一版按同型号 2804 处理：

| 参数 | Motor 1 | Motor 2 |
|---|---:|---:|
| 极对数 | 7 | 7 |
| 额定电压 | 12 V | 12 V |
| 初始额定电流 | 0.4 A | 0.4 A |
| 初始软件限流 | 1.0 A | 1.0 A |
| 初始相电阻 | 2.3 Ω | 2.3 Ω |
| 初始相电感 | 0.86 mH | 0.86 mH |

0.4 A/1.0 A、2.3 Ω 和 0.86 mH 采用与“2804 + AS5600 + 12 V + 7 极对”最匹配的公开型号作为启动基线；正式量产参数必须以实测值覆盖。

## 3. 设计原则

1. 两个电机必须拥有独立的控制上下文、参数、状态、故障码和遥测数据。
2. 电流采样和 PWM 必须由硬件定时关系保证，不能依赖主循环轮询。
3. 电流环、速度环、位置环分离，分别拥有固定执行周期。
4. 任何 PWM 输出都必须有明确的关闭路径。
5. 参数、硬件接口和控制算法解耦，后续更换 MCU 或驱动芯片时不修改 FOC 数学层。
6. 第一版优先完成带 AS5600 的有感 FOC，再实现无感、弱磁和 MTPA。
7. 所有关键限制值必须可配置，并在运行状态中可读取。

## 4. 软件分层

```text
application/
├── motor_app.c              启动、停止、命令和系统策略
├── motor_state_machine.c    双电机状态机
└── motor_protocol.c         串口调试协议

control/
├── foc_core.c               单个电机 FOC 主流程
├── foc_transform.c          Clarke/Park/反变换
├── foc_svpwm.c              SVPWM
├── foc_pi.c                 Id/Iq/速度 PI
├── motor_observer.c         后续无感观测器接口
└── motor_feedback.c         位置和速度反馈处理

hardware/
├── motor_pwm.c              TIM1/TIM3 三路 PWM
├── motor_adc.c              ADC1 注入/常规组和数据换算
├── motor_encoder.c          两个 AS5600 I²C 驱动
├── motor_fault.c            软件故障管理
└── motor_hw_config.c        引脚、比例和硬件参数

common/
├── motor_types.h            公共类型和枚举
├── motor_config.h           默认参数
└── motor_telemetry.c        运行数据快照
```

所有控制算法函数必须接收 `motor_control_t *motor`，禁止使用只属于某一电机的隐式全局变量。

## 5. 双电机数据模型

```c
typedef struct
{
    float ia;
    float ib;
    float ic;
    float id;
    float iq;
    float vbus;
    float electrical_angle;
    float mechanical_angle;
    float mechanical_speed_rpm;
} motor_measurements_t;

typedef struct
{
    float id_ref;
    float iq_ref;
    float speed_ref_rpm;
    float position_ref_rad;
} motor_references_t;

typedef struct
{
    motor_params_t params;
    motor_measurements_t measure;
    motor_references_t reference;
    motor_feedback_t feedback;
    motor_state_t state;
    motor_fault_t fault;
    pi_controller_t id_pi;
    pi_controller_t iq_pi;
    pi_controller_t speed_pi;
} motor_control_t;
```

系统维护：

```c
motor_control_t g_motor[2];
adc_scheduler_t g_adc_scheduler;
```

## 6. 控制时序需求

### 6.1 快速电流环

每个电机目标执行频率为 20 kHz：

```text
PWM 周期事件
  → 对应 ADC 采样
  → 电流偏置和比例换算
  → Ia/Ib/Ic 重构
  → Clarke
  → Park
  → Id/Iq PI
  → 反 Park
  → SVPWM
  → 更新对应定时器 CCR1/CCR2/CCR3
```

快速中断中禁止：

- I²C 访问
- UART 发送
- Flash 写入
- 阻塞等待
- 动态内存分配
- 大量日志输出

### 6.2 ADC 共享调度

ADC 管理器必须提供：

- 当前采样归属：Motor 1 或 Motor 2
- 注入组/常规组转换完成标志
- 采样超时检测
- 采样冲突计数
- 转换完成时间戳或周期计数
- ADC 错误恢复流程

两个电机的采样触发必须在 PWM 周期内留出足够的采样窗口，不能仅依据“两个定时器频率相同”推定安全。最终必须用示波器或逻辑分析仪验证 PWM、ADC 触发和采样输出之间的时序。

### 6.3 慢速环

- 速度环：1 kHz
- 位置环：500 Hz～1 kHz
- 温度/状态监控：100 Hz～500 Hz
- 通信遥测：100 Hz～500 Hz

## 7. FOC 功能需求

### P0：首版必须完成

- 双电机独立 PWM 启停
- 两电机电流 ADC 读取和偏置校准
- 两相电流转实际安培值
- 第三相电流重构
- Clarke/Park/反 Park 变换
- SVPWM
- 两套 Id/Iq PI 控制器
- AS5600 角度读取
- 机械角到电角度转换：`θe = 7 × θm + θoffset`
- 转子初始对齐
- 转矩/电流控制模式
- 独立启动、停止、故障状态机
- 软件电流限幅
- 串口读取状态、角度、电流和故障码

### P1：第二阶段完成

- 速度闭环
- 速度滤波和估算
- AS5600 断线/无效检测
- 启动失败检测
- 堵转检测
- 双电机同步运行压力测试
- 参数保存和加载
- 上位机实时变量监视

### P2：后续扩展

- 无感观测器
- 无感启动和有感/无感切换
- 弱磁控制
- MTPA
- 电压前馈和交叉耦合补偿
- 参数自动辨识
- 位置闭环

## 8. 状态机需求

每个电机拥有独立状态机：

```text
RESET
  ↓
INIT
  ↓
ADC_OFFSET_CALIBRATION
  ↓
READY
  ↓
ALIGN
  ↓
OPEN_LOOP_START
  ↓
CURRENT_CONTROL
  ↓
SPEED_CONTROL
  ↓
STOPPING
  ↓
READY
```

任意运行状态都可以进入：

```text
FAULT → PWM_OFF → FAULT_LATCHED
```

Motor 1 的普通故障不强制 Motor 2 停止；母线、电源和驱动器级故障由系统策略统一关闭两个电机。

## 9. 保护需求

由于本版本不使用母线电压 ADC，MCU 侧不实现实时母线过压/欠压判断。以下功能由外部电源或驱动芯片负责：

- 母线过压
- 母线欠压
- 功率级硬件过流
- 驱动器死区
- 栅极互锁

MCU 软件必须实现：

- 相电流超过 1.0 A 时停止对应电机
- ADC 转换超时
- ADC 数据越界
- AS5600 无响应或角度跳变
- 启动失败
- 堵转
- 速度超过配置上限
- FOC 快速环执行超时
- PWM 更新丢失

停止动作必须先将对应 CCR 设置为安全占空比，再停止 PWM 输出。故障码必须锁存，清除故障前不得自动重新启动。

## 10. 接口需求

建议提供以下 API：

```c
bool motor_control_init(uint8_t motor_id);
bool motor_start(uint8_t motor_id);
bool motor_stop(uint8_t motor_id);
bool motor_clear_fault(uint8_t motor_id);
bool motor_set_iq_ref(uint8_t motor_id, float iq_ref);
bool motor_set_speed_ref(uint8_t motor_id, float speed_rpm);
motor_state_t motor_get_state(uint8_t motor_id);
motor_fault_t motor_get_fault(uint8_t motor_id);
const motor_measurements_t *motor_get_measurements(uint8_t motor_id);
void motor1_fast_loop(void);
void motor2_fast_loop(void);
void motor_slow_loop_1khz(void);
```

## 11. 参数和校准需求

上电后必须支持：

1. ADC 零电流偏置校准
2. AS5600 在线检测
3. 角度方向检测
4. 电角度零位对齐
5. 相序和电流方向确认
6. PI 初始参数加载

电流换算采用：

```text
I = (Vout - Voffset) / (50 × 0.005)
I = (Vout - Voffset) / 0.25
```

`Voffset` 必须通过启动校准获得，不将 1.65 V 写死为唯一值。

## 12. 验收标准

### 12.1 软件构建

- Keil 工程可完整编译
- 无新增编译警告，或每个新增警告有明确原因
- 双电机模块不修改 CubeMX 生成区之外的无关功能

### 12.2 PWM

- TIM1 和 TIM3 均为中心对齐模式
- 两路 PWM 频率均约 20 kHz
- 启动前三路输出为安全电平
- 停止后占空比归零或进入驱动器规定的安全状态
- 两电机占空比互不覆盖

### 12.3 ADC 与电流

- 零电流校准后，静态电流误差满足硬件噪声水平
- 正负电流方向经过实测确认
- 两个电机同时运行时无 ADC 冲突计数增长
- 每个采样周期都能对应正确的电机实例
- 相电流重构满足 `Ia + Ib + Ic ≈ 0`

### 12.4 AS5600

- 两个 I²C 总线均能识别 `0x36`
- 角度连续旋转时无明显跳变
- 机械角方向和电角度方向一致
- 断开任一 AS5600 后，对应电机进入故障状态

### 12.5 FOC 运行

- 单电机可独立完成对齐、启动、转矩控制和停止
- 双电机可同时运行
- `Iq` 变化能对应实际转矩变化
- 电流参考值变化时，电流环不持续积分饱和
- 快速环执行时间低于一个控制周期，并保留实测计数

### 12.6 故障

- 软件电流超过 1.0 A 后对应电机停止
- ADC 超时、编码器断线和启动失败可重复触发
- 故障码可通过串口读取
- 故障清除前不会自动重新启动

## 13. 实现顺序

1. 固化双电机类型、参数和引脚映射
2. 完成 Motor 1/Motor 2 PWM 抽象
3. 完成 ADC 双通道采样和零点校准
4. 完成 AS5600 两路 I²C 驱动
5. 完成角度、相序和电流方向验证
6. 完成数学变换和单电机开环电压矢量
7. 完成单电机电流环
8. 完成双电机电流环和 ADC 调度
9. 完成转矩控制和故障状态机
10. 完成速度闭环、通信和参数保存

## 14. 参考成熟方案

- [ST STM32 Motor Control Ecosystem](https://www.st.com/content/st_com/en/ecosystems/stm32-motor-control-ecosystem.html)：双电机、共享 ADC、1/3 分流采样、位置反馈和保护能力
- [ST STM32F PMSM single/dual FOC SDK user manual](https://www.st.com/resource/en/user_manual/um1052-stm32f-pmsm-singledual-foc-sdk-v43-stmicroelectronics.pdf)：PWM 与 ADC 同步、两相采样重构和状态机
- [TI MotorControl SDK](https://software-dl.ti.com/C2000/docs/software_guide/mcsdk.html)：电流环、速度环、位置反馈、启动失败、堵转和参数辨识的完整组织方式
- [INA240 官方数据手册](https://www.ti.com/lit/ds/symlink/ina240.pdf)：INA240A2 50 V/V、低端双向电流检测和 PWM 抑制特性
- [AS5600 官方数据手册](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)：I²C 地址、角度寄存器和 12 bit 角度数据
- [DFRobot 2804 + AS5600 官方规格](https://wiki.dfrobot.com/fit1034/)：12 V、7 极对、0.4 A 额定电流和 1 A 最大电流的匹配型号基线

## 15. 当前明确不纳入第一版的内容

- ESP32、Arduino、ESP-IDF、FreeRTOS
- 6PWM 互补输出
- MCU TIM1/TIM3 Break 保护输入
- MCU 母线电压 ADC 采样
- 无感启动
- 弱磁、MTPA 和高级参数辨识
- 双电机共享一个 AS5600 I²C 总线
