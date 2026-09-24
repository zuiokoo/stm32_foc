"""iq 电流环调试脚本（通过 COM7）。
步骤：关 VOFA -> 读基线 -> 设参数 -> STOP/ALIGN/RUN -> 分档给 iq 给定并轮询 STATUS -> 汇总 -> STOP。
只做小电流（<= 0.1A 的 iq，d 轴 0~0.2A 方波），末尾一定 STOP。
"""
import re, statistics as st, time, serial

PORT = 'COM7'
iq_steps = [0.0, 0.05, 0.10, 0.0]     # 每档给 2s
s = serial.Serial(PORT, 115200, timeout=0.15)

def send(cmd, wait=0.35):
    s.reset_input_buffer()
    s.write((cmd + '\n').encode())
    time.sleep(wait)
    return s.read(8000).decode('ascii', 'replace')

def one_status(timeout=0.6):
    """发一条 STATUS，解析 ANGLE/ID/IQ；失败返回 None"""
    s.reset_input_buffer()
    s.write(b'STATUS\n')
    t0 = time.time()
    buf = ''
    while time.time() - t0 < timeout:
        buf += s.read(4096).decode('ascii', 'replace')
        m = re.search(r'ANGLE ([-\d.]+)\r?\nID ([-\d.]+)\r?\nIQ ([-\d.]+)', buf)
        if m:
            return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None

# 1) 关掉 VOFA 二进制流，让串口只剩 ASCII
print('== VOFA OFF ==');  print(send('VOFA OFF', 0.8))
print('== baseline STATUS =='); print(send('STATUS'))

# 2) 参数（只改变量，不驱动）
send('PID_D 6 20000'); send('PID_Q 6 20000'); send('ID 0.0'); send('IQ 0.0')
print('== params set: PID_D 6 20000 / PID_Q 6 20000 ==')

# 3) STOP -> ALIGN -> RUN，逐步确认
print('== STOP ==');  print(send('STOP'))
print('== ALIGN =='); r = send('ALIGN', 1.2); print(r)
print('== RUN ==');   r = send('RUN', 0.8); print(r)

got = one_status()
print('after RUN, first STATUS =', got)
if got is None:
    print('!! STATUS 无响应，脚本中止'); s.close(); raise SystemExit(3)

# 4) 分档给 iq 给定，每档轮询 2s
seg = {}
for iq in iq_steps:
    send('IQ %.2f' % iq, 0.3)
    time.sleep(0.4)                      # 等它稳
    A, I, Q = [], [], []
    t0 = time.time()
    while time.time() - t0 < 2.0:
        r = one_status()
        if r:
            A.append(r[0]); I.append(r[1]); Q.append(r[2])
        # 越快越好，不加额外延时
    if I:
        seg[iq] = dict(n=len(I), ang=A[0], ang_end=A[-1],
                       id=st.mean(I), iq=st.mean(Q),
                       iq_sd=st.pstdev(Q) if len(Q) > 1 else 0.0,
                       id_sd=st.pstdev(I) if len(I) > 1 else 0.0)
        print('IQ_ref=%.2f  n=%d  id=%.4f(±%.4f)  iq=%.4f(±%.4f)  angle %.3f->%.3f'
              % (iq, len(I), seg[iq]['id'], seg[iq]['id_sd'],
                 seg[iq]['iq'], seg[iq]['iq_sd'], seg[iq]['ang'], seg[iq]['ang_end']))
    else:
        print('IQ_ref=%.2f  无有效STATUS响应' % iq)

print('== STOP =='); print(send('STOP'))
print('== 汇总 ==')
for iq, v in seg.items():
    print('  IQ_ref=%.2f -> iq=%.4f (目标 %.2f, 误差 %+.4f)  id=%.4f  angle漂移 %+.3f rad'
          % (iq, v['iq'], iq, v['iq'] - iq, v['id'], v['ang_end'] - v['ang']))
s.close(); print('CLOSED_OK')
