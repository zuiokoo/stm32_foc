"""烧写后第一轮：ALIGN(验证 motor_pwm 修复) -> RUN -> d/q 分档测试。
安全：id/iq <= 0.1A，末尾 STOP。全程打印原始响应。"""
import re, statistics as st, time, serial

s = serial.Serial('COM7', 115200, timeout=0.15)

def send(cmd, wait=0.5):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(9000).decode('ascii', 'replace')

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

print('== drain ==');        print(send('', 0.8))
print('== ALIGN ==');        print(send('ALIGN', 1.5))
print('== STATUS(对齐后) =='); print(send('STATUS', 0.4))
send('PID_D 6 20000'); send('PID_Q 6 20000'); send('ID 0.0'); send('IQ 0.0')
print('== RUN ==');          print(send('RUN', 1.0))

print('== 基线(两轴给定0，验证环路已运行) =='); print(sample(1.2))

print('== ID 0.10（d 轴：不产生力矩，同时验证对齐零位）==')
send('ID 0.10', 0.3); time.sleep(0.4); r1 = sample(1.5); print(r1)

print('== ID 0.0 / IQ 0.05（q 轴）==')
send('ID 0.0', 0.2); send('IQ 0.05', 0.3); time.sleep(0.4); r2 = sample(1.5); print(r2)

print('== IQ 0.10 ==')
send('IQ 0.10', 0.3); time.sleep(0.4); r3 = sample(1.5); print(r3)

print('== IQ 0.0 ==')
send('IQ 0.0', 0.3); time.sleep(0.4); r4 = sample(1.0); print(r4)

print('== STOP =='); print(send('STOP', 0.5))
print('== 汇总 ==')
for name, v, tgt, key in [('ID=0.10', r1, 0.10, 'id'), ('IQ=0.05', r2, 0.05, 'iq'), ('IQ=0.10', r3, 0.10, 'iq')]:
    if v:
        print('  %-8s 目标 %.2f -> 实测 %+.4f (误差 %+.4f, 波动 ±%.4f) | id=%+.4f iq=%+.4f | 角度漂移 %+.3f rad'
              % (name, tgt, v[key], v[key] - tgt, v[key + '_sd'], v['id'], v['iq'], v['a1'] - v['a0']))
s.close(); print('CLOSED_OK')
