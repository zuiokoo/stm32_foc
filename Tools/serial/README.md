# 串口调试脚本（COM7 @115200，pyserial）

配合固件的 ASCII 命令行走，用来标定/验证电流环。发命令前先 `VOFA OFF`，
避免 JustFloat 二进制流混进 ASCII 解析。

## 脚本

| 脚本 | 用途 |
|---|---|
| `enc_probe.py` | 只读探测：抓一段日志 + 发 `STATUS`，确认在线、看编码器/状态 |
| `diag_all.py` | 汇总诊断：ISR 频率、编码器失败计数、I2C 引脚电平、状态机 |
| `iq_tune.py` / `iq_tune2/3/4.py` | q 轴阶跃（锁轴），扫 kp/ki，读稳态误差与超调 |
| `iq_verify_locked.py` | 锁轴下的正式验证：多电流点稳态 + 阶跃动态 |
| `kp_sweep.py` | 固定 ki 扫 kp，看超调/上升时间 |
| `cap_step.py` | 用固件 RAM 抓取（`CAP`）测 50us 级动态：发 `CAP` 再改给定，读 `CAP BEGIN/C/END` |

## 固件侧命令

```
ALIGN            RUN              STOP             CLEAR（清故障）
ID <A>           IQ <A>           STATUS
PID_D <kp> <ki>  PID_Q <kp> <ki>
CAP                                          抓 400 点 × 50us
VOFA ON/OFF      OPENLOOP / CLOSELOOP         OL_SPEED <x>  OL_VOLT <V>
HELP
```

## 当前标定值（2026-09-24 实测）

| 项 | 值 | 依据 |
|---|---|---|
| `PID_D` / `PID_Q` | kp = 2, ki = 10000 | 固件上电默认（main.c），CLI 可临时覆盖 |
| 零点对齐 | 自动（`ALIGN`：等转子静止后再读零点） | 之前固定 500ms 读会造成几十度偏差 |
| 相电阻 R | 3.4 Ω | 锁轴 vq = 0.34 V @ iq = 0.1 A |
| 相电感 L | 0.68 mH | 电流上升沿拟合 |
| ki/kp | 0.2 ms = L/R | 零极点对消 |
| 电流环带宽 | ~200 Hz（kp=2 时超调约 25%） | d 轴阶跃：上升 0.25~0.30 ms，稳态误差 <1% |

注意：`IQ` 的动态标定要在**刚性固定转子**下做。转子自由时反电势和转动都会
污染阶跃波形（实测角度漂 1 rad、超调虚高到 56%）。
