# FOC 成熟方案架构对照附录

## 1. 对照结论

本项目不直接移植 VESC、espFoC 或 SimpleFOC 的运行时，而是提取它们已经验证过的架构边界，再适配当前 STM32F401 双电机硬件。

| 方案 | 核心架构 | 本项目吸收的内容 | 本项目不直接采用的内容 |
|---|---|---|---|
| VESC | `motor`、`driver`、`encoder`、`comm`、`applications`、`hwconf` 分层 | 模块边界、板级配置分离、统一电机接口、参数配置 | ChibiOS、多板目标、整车级通信、电池管理 |
| espFoC | 一个 Axis 由逆变器、转子传感器、电流传感器和电机设置组成 | `motor_axis` 实例、快速 ISR/慢速调节分离、运行时调参、测试/标定分离 | ESP-IDF、FreeRTOS、Q16 固定点实现 |
| SimpleFOC | 精简算法核心 + 可替换传感器/驱动/电流采样 | 驱动接口、传感器接口、电流采样接口、零点校准、独立测试例程 | Arduino 类体系、Arduino 运行时 |
| ST MCSDK | PWM-ADC 同步、双电机共享 ADC、状态机、保护 | 双电机实时调度、采样时序、故障管理、启动流程 | MC Workbench 生成器和厂商专用库 |
| TI MotorControl SDK | 电流环、速度环、位置反馈、参数辨识和保护完整组合 | 环路分层、参数结构、调参和保护需求 | C2000 专用外设和 FAST 观测器 |

## 2. VESC 的可迁移部分

VESC 源码将公共接口、FOC 数学、PWM/ADC、电机配置、编码器、通信和应用拆分在不同目录中；电机目录中还单独区分 `foc_math`、`mcpwm_foc` 和 `mc_interface`。

因此本项目必须避免出现一个巨大的 `main.c`，采用以下边界：

```text
motor_control       电机公共 API 和状态
foc_math             Clarke/Park/PI/SVPWM
motor_pwm            TIM1/TIM3 三路 PWM
motor_adc            ADC1 采样和转换
motor_encoder       AS5600 读取、角度和速度
motor_config         两套电机参数
motor_protocol       串口命令和遥测
motor_fault          故障记录和停机策略
```

每个模块只暴露接口，不跨层访问另一个模块的内部变量。

## 3. espFoC 的可迁移部分

espFoC 的重要设计不是 ESP32，而是 Axis 抽象：一个 Axis 绑定一套逆变器、位置传感器、电流传感器和参数；快速路径只做实时控制，慢速路径负责传感器读取、调节和外部控制。

本项目对应为：

```c
motor_axis_t motor_axis[2];

motor_axis[0] = {
    .pwm = TIM1,
    .current = ADC1_INJECTED,
    .sensor = I2C1_AS5600,
};

motor_axis[1] = {
    .pwm = TIM3,
    .current = ADC1_REGULAR_DMA,
    .sensor = I2C2_AS5600,
};
```

快速路径只允许：

```text
读取已完成的 ADC 数据
→ 电流换算
→ Clarke/Park
→ Id/Iq PI
→ 反 Park/SVPWM
→ 更新 PWM
```

I²C 读取 AS5600 不得放入 20 kHz PWM/ADC 中断，而应放在慢速任务中，并通过角度插值或速度估算提供快速环使用的电角度。

## 4. SimpleFOC 的可迁移部分

SimpleFOC 将硬件差异集中在驱动、传感器和电流采样模块；其电流采样流程包含 ADC 配置、零点校准、采样同步、相电流读取和 D/Q 电流计算。

本项目的接口应保持同样的可替换关系：

```text
FOC Core
 ├── CurrentSensor interface
 ├── PositionSensor interface
 └── PwmDriver interface
```

当前实现：

```text
CurrentSensor → INA240A2PW + 5mΩ + ADC1
PositionSensor → AS5600 + I2C1/I2C2
PwmDriver → TIM1/TIM3 3PWM
```

必须提供不接电机的独立测试模式：

- PWM 三相输出测试
- ADC 零点测试
- 单路电流方向测试
- AS5600 角度连续性测试
- 相序测试
- 电流采样与电角度对齐测试
- 开环电压矢量测试

## 5. 对当前双电机工程的最终收敛架构

```text
                    motor_app
                       │
             ┌─────────┴─────────┐
             │                   │
        motor_axis[0]       motor_axis[1]
             │                   │
       ┌─────┼─────┐       ┌─────┼─────┐
       │     │     │       │     │     │
     PWM   ADC   AS5600   PWM   ADC   AS5600
     TIM1  注入   I2C1    TIM3  常规   I2C2
       │     │     │       │     │     │
       └─────┴─────┘       └─────┴─────┘
              │                   │
              └───────┬───────────┘
                      │
               adc_resource_mgr
```

## 6. 强制实时规则

### 快速环

- Motor 1 和 Motor 2 目标频率均为 20 kHz
- 使用定时器同步 ADC 采样
- 快速中断不得调用 I²C、UART、Flash 或阻塞函数
- 快速环必须使用静态内存
- 必须统计最大执行周期、平均执行周期和丢采样次数
- 每个电机必须使用独立的 ADC 缓冲、PI 状态和 PWM 更新变量

### 慢速环

- AS5600 读取建议在 1～2 kHz 调度
- 速度环 1 kHz
- 状态机和故障处理 1 kHz
- 遥测 100～500 Hz
- 参数保存只允许在电机停止状态执行

## 7. 需要对原需求文档做的架构修订

原需求文档中的模块设计保持有效，但增加以下硬性要求：

1. `motor_control_t` 必须升级为两个完整的 `motor_axis_t` 实例。
2. ADC 不能被 Motor 1 和 Motor 2 直接独立启动，必须由 ADC 资源管理器仲裁。
3. AS5600 的 I²C 读取必须与 FOC 快速环解耦。
4. FOC 核心必须支持无传感器模拟角度输入，便于开环和单元测试。
5. PWM、ADC、电流采样、编码器和控制算法必须可以单独测试。
6. 量产前必须实测电流方向、相序、电角度偏置和 ADC 采样时序。
7. 首版使用浮点实现以降低调试复杂度；只有实测 CPU 周期不足时才引入定点优化。

## 8. 参考来源

- [VESC bldc firmware](https://github.com/vedderb/bldc)
- [VESC motor modules](https://github.com/vedderb/bldc/tree/master/motor)
- [espFoC](https://github.com/uLipe/espFoC)
- [SimpleFOC Drivers](https://docs.simplefoc.com/drivers_library)
- [SimpleFOC Current Sensing](https://docs.simplefoc.com/current_sense)
- [ST STM32 Motor Control Ecosystem](https://www.st.com/content/st_com/en/ecosystems/stm32-motor-control-ecosystem.html)
- [TI MotorControl SDK](https://software-dl.ti.com/C2000/docs/software_guide/mcsdk.html)

