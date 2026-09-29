# 串口调试脚本（COM7 @115200，pyserial）

配合固件 CLI 做标定/验证。**命令与判据的完整说明见 [`../../docs/foc-tuning-notes.md`](../../docs/foc-tuning-notes.md)**
（那里有每个参数的实测数据和"怎么判断"的方法）。

发命令前建议先 `VOFA OFF`，否则二进制 JustFloat 帧会插进 ASCII 回复里（表现为 `SPD_EN 1A` 这种错乱）。

## 脚本

| 脚本 | 用途 |
|---|---|
| `enc_probe.py` | 只读探测：抓日志 + 发 `STATUS`，确认在线 |
| `diag_all.py` | 汇总诊断：ISR 频率、编码器失败计数、I2C 引脚电平、状态机 |
| `ripple_diag.py` | 电流环纹波溯源（`CAP` 50µs × 400 点） |
| `cap_save.py` | CAP 抓取并存 CSV，同时用"隐含反电势"判断转子是否静止 |
| `cap_analyze.py` | CAP + 频域分析（谱峰、白噪声判据、vq×iq 互相关） |
| `ma8_check.py` | 用"环路关"对照测纯测量链底噪（判断带宽是不是采样噪声） |
| `gain_test.py` | 增益置 0/半值/标定值对比，定位振荡来源 |
| `extrap_verify.py` | 验证电角度外推（低电流自由转动下测角度质量） |
| `speed_start.py` | 速度环第一轮：外推验证 + SPD 10/30 阶跃（带发散保护） |
| `speed_test.py` | 速度环阶跃 + 响应分析（上升/超调/稳态误差） |
| `speed_sweep.py` `speed_sweep2.py` | 速度环参数扫描（第一轮扫 ki、第二轮扫 kp） |
| `decode_frames.py` | 解析手抄的 VOFA JustFloat 帧（含 LSB 格点检查） |
| `iq_tune*.py` `iq_verify_locked.py` `kp_sweep.py` `cap_step.py` | 早期电流环标定脚本 |

`frames_1704.txt` / `frames_1717.txt` 是两台固件的 VOFA 原始帧样本（MA4 / MA8 对比用）。

## 固件侧命令（简表）

```
ALIGN            RUN              STOP             CLEAR（清故障，并清外环使能）
ID <A>           IQ <A>           STATUS
PID_D/PID_Q <kp> <ki>             PID_S <kp> <ki>  PID_P <kp> <ki>
SPD <rad/s>      SPD_OFF          FF <A>           POS <rad>        POS_OFF
CAP （电流环 50µs×400=20ms）      CAPS （速度/位置环 5ms×400=2s）
VOFA ON/OFF      OPENLOOP / CLOSELOOP         OL_SPEED <x>  OL_VOLT <V>
HELP
```

**注意**：`SPD`/`POS` 只是设给定 + 使能，真正执行的是 20kHz ISR 里的环路，
而 ISR 只在 `RUNNING`（STATUS 里 `STATE 3`）状态跑 —— **必须先 `RUN`**。

## 典型流程

```bash
python Tools/serial/enc_probe.py        # 确认在线
python Tools/serial/gain_test.py        # 分离"测量噪声"与"环路振荡"
python Tools/serial/extrap_verify.py    # 验电角度外推
python Tools/serial/speed_sweep2.py     # 扫速度环 kp
```
