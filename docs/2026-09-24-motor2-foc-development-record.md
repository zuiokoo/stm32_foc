# Motor2 FOC 开发记录

日期：2026-09-24
工程：`stm32_foc`

## 1. 当前目标

当前工程以 Motor2 为主，目标是跑通：

```text
ADC 同步采样 → 电流换算 → Clarke → Park → d/q PI → 逆 Park → SVPWM → TIM1 三路 PWM
```

当前已经可以进行开环运行和波形观测；闭环电流的目标跟踪、采样极性和相序仍需用固定电压矢量进一步确认。

## 2. Motor2 硬件与定时器配置

| 功能 | 当前配置 |
|---|---|
| PWM 定时器 | TIM1 |
| PWM 输出 | TIM1_CH1/CH2/CH3，PA8/PA9/PA10 |
| PWM 计数模式 | 中心对齐模式 1 |
| PWM 周期 | `2100-1` |
| PWM 频率 | 约 20 kHz |
| ADC 触发 | TIM1_CH4 比较事件 |
| CH4 比较值 | `1900-1` |
| ADC 触发沿 | 上升沿 |
| ADC 注入 Rank1 | ADC1_IN5 / PA5 → `current_a` |
| ADC 注入 Rank2 | ADC1_IN4 / PA4 → `current_b` |
| 电流放大器 | INA240A2，增益 50 V/V |
| 分流电阻 | 5 mΩ |
| 母线电压 | 当前算法固定使用 12 V |

当前 ADC 注入顺序为：

```text
Rank1 = PA5 = ADC_CHANNEL_5
Rank2 = PA4 = ADC_CHANNEL_4
```

`HAL_ADCEx_InjectedStart_IT(&hadc1)` 在初始化阶段启动一次，之后由 TIM1_CH4 周期触发注入转换，ADC 注入转换完成回调中执行 Motor2 快速环。

## 3. 电流换算

```text
ADC 电压比例 = 3.3 / 4095
功率级电流比例 = 5 mΩ × 50 = 0.25 V/A
理论比例 ≈ 0.003223 A/count
```

当前 `Hardware/current.c` 使用：

```c
#define CURRENT_SENSE_SIGN (-1.0f)
```

并对 A、B 两相电流统一乘以该符号，C 相使用：

```c
current_c = -current_a - current_b;
```

当前符号是工作区中的调试设置，最终应使用固定正电压矢量测试和 INA240 `IN+/IN-` 方向共同确认，不能仅凭 Kp 正负决定。

## 4. FOC 数学链路

两电阻采样使用：

```text
i_alpha = i_a
i_beta  = (i_a + 2*i_b) / sqrt(3)
```

Park 变换：

```text
i_d =  i_alpha*cos(theta) + i_beta*sin(theta)
i_q = -i_alpha*sin(theta) + i_beta*cos(theta)
```

电流环误差为：

```c
error_d = id_ref - id;
error_q = iq_ref - iq;
```

PI 调用周期固定为：

```text
dt = 0.00005 s = 50 us
```

当 `id_ref=0` 且 `PID_D 2 0` 时：

```text
vd = 2 × (0 - id) = -2 × id
```

因此 VOFA 中 `vd` 与 `id` 反向是该 P 控制关系的正常结果；判断闭环方向时应观察 `id_ref=+0.2` 时实际 `id` 是否跟踪到 `+0.2`。

## 5. 位置传感器与状态机

- 传感器：AS5600
- 主循环读取周期：约 5 ms
- 电机极对数：7
- 电角度：机械角度 × 极对数 − 电角度零点偏置
- 对齐电压：0.5 V
- 对齐时间：500 ms

状态机：

```text
CALIBRATING → ALIGN → READY → RUNNING
                         ↘ OPENLOOP
任意状态 → FAULT
```

## 6. 当前开环与闭环状态

当前默认开环参数：

```c
motor2_openloop_speed   = 0.05f;
motor2_openloop_voltage = 1.0f;
motor2_openloop_enable  = 1;
```

开环路径在 ADC 注入回调中生成电角度，并使用：

```text
vd = 0
vq = motor2_openloop_voltage
```

当前已用于验证 PWM、ADC 触发和电流波形链路。闭环电流环目前不能记录为“已完全通过”，还需要完成固定电压矢量测试：

1. 关闭 PI，固定 `vd=+0.3 V、vq=0`；
2. 使用固定或已知电角度；
3. 观察 `id` 和 `iq`；
4. `id>0、iq≈0`：电流符号和相序正确；
5. `id<0、iq≈0`：电流采样极性相反；
6. `id` 与 `iq` 同时明显混合：采样相与 PWM U/V 相映射不一致。

## 7. 串口和 VOFA

USART2 接收采用逐字节中断，命令在主循环解析和发送。当前命令包括：

```text
ALIGN
RUN
STOP
ID x
IQ x
PID_D kp ki
PID_Q kp ki
STATUS
VOFA ON
VOFA OFF
OPENLOOP
CLOSELOOP
OL_SPEED x
OL_VOLT x
HELP
```

VOFA 7 通道：

| 通道 | 数据 |
|---:|---|
| 1 | `motor2_id_ref` |
| 2 | `motor2_id` |
| 3 | `motor2_vd` |
| 4 | `motor2_iq` |
| 5 | `current_a` |
| 6 | `current_b` |
| 7 | `current_c` |

## 8. 当前调试结论

- 开环运行链路已经建立，能够输出三相 PWM 并观察电流波形。
- ADC 注入 Rank 已按当前硬件定义为 PA5、PA4。
- FOC 电流环误差使用“参考值 − 反馈值”，该公式没有反向。
- `vd` 与 `id` 在 `id_ref=0、Ki=0` 时反向，符合 P 负反馈公式。
- `id_ref=+0.2` 时实际 `id` 若趋向 `-0.2`，则闭环的电压坐标和电流反馈坐标仍未对齐。
- 当前 `CURRENT_SENSE_SIGN=-1.0f` 是工作区中的调试设置；最终仍需固定电压矢量测试确认。

## 9. Git 提交范围

本次记录包含当前源代码、CubeMX 工程配置和本记录文档。以下属于本地调试产物，不提交：

```text
.workbuddy/
MDK-ARM/JLinkLog.txt
MDK-ARM/JLinkSettings.ini
```