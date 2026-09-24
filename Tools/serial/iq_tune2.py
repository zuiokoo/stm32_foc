"""iq 电流环调试（第二版）：先 CLEAR 解除 FAULT 锁死，再 RUN，然后 d 轴与 q 轴分档测试。
安全：iq <= 0.1A、id <= 0.1A，末尾 STOP。"""
import re, statistics as st, time, serial

s = serial.Serial('COM7', 115200, timeout=0.15)

def send(cmd, wait=0.4):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(8000).decode('ascii', 'replace')

def one_status(timeout=0.6):
    s.reset_input_buffer(); s.write(b'STATUS\n')
    t0 = time.time(); buf = ''
    while time.time() - t0 < timeout:
        buf += s.read(4096).decode('ascii', 'replace')
        m = re.search(r'ANGLE ([-\d.]+)\r?\nID ([-\d.]+)\r?\nIQ ([-\d.]+)', buf)
        if m: return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None

def sample(secs=1.5):
    A, I, Q = [], [], []
    t0 = time.time()
    while time.time() - t0 < secs:
        r = one_status()
        if r: A.append(r[0]); I.append(r[1]); Q.append(r[2])
    if not I: return None
    return dict(n=len(I), id=st.mean(I), iq=st.mean(Q),
                id_sd=st.pstdev(I) if len(I) > 1 else 0, iq_sd=st.pstdev(Q) if len(Q) > 1 else 0,
                a0=A[0], a1=A[-1])

# 1) 清故障 -> READY
print('== CLEAR ==');  print(send('CLEAR', 0.6))
# 2) 参数
send('PID_D 6 20000'); send('PID_Q 6 20000'); send('ID 0.0'); send('IQ 0.0')
# 3) 进 RUN
print('== RUN ==');    print(send('RUN', 0.8))
time.sleep(0.3)

print('== 基线(两轴给定 0) ==')
b = sample(1.2); print(b)

print('== ID 0.10（d 轴，不产生力矩）==')
send('ID 0.10', 0.3); time.sleep(0.4)
d1 = sample(1.5); print(d1)

print('== ID 0.0, IQ 0.05（q 轴）==')
send('ID 0.0', 0.2); send('IQ 0.05', 0.3); time.sleep(0.4)
q1 = sample(1.5); print(q1)

print('== IQ 0.10 ==')
send('IQ 0.10', 0.3); time.sleep(0.4)
q2 = sample(1.5); print(q2)

print('== IQ 0.0 ==')
send('IQ 0.0', 0.3); time.sleep(0.4)
q0 = sample(1.0); print(q0)

print('== STOP =='); print(send('STOP', 0.5))
print('== 汇总 ==')
for name, v in [('ID=0.10', d1), ('IQ=0.05', q1), ('IQ=0.10', q2)]:
    if v:
        tgt = 0.10 if name.startswith('ID') else float(name.split('=')[1])
        got = v['id'] if name.startswith('ID') else v['iq']
        print('  %-8s 目标 %.2f -> 实测 %.4f (误差 %+.4f, 波动 ±%.4f)  id=%.4f iq=%.4f 角度漂移 %+.3f'
              % (name, tgt, got, got - tgt, v['iq_sd'] if name.startswith('IQ') else v['id_sd'],
                 v['id'], v['iq'], v['a1'] - v['a0']))
s.close(); print('CLOSED_OK')
