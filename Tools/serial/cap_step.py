"""动态验证：用 ISR 内 RAM 抓取(50us/点, 400点=20ms)看 iq / id 的阶跃响应。
抓取通道(ASCII 行 "C idx id_ref iq_ref id iq vd vq")。"""
import re, time, serial

s = serial.Serial('COM7', 115200, timeout=0.2)
DT = 50e-6

def send(cmd, wait=0.5):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(9000).decode('ascii', 'replace')

def status():
    s.reset_input_buffer(); s.write(b'STATUS\n')
    t0 = time.time(); buf = ''
    while time.time() - t0 < 1.0:
        buf += s.read(4096).decode('ascii', 'replace')
        m = re.search(r'ANGLE ([-\d.]+)\r?\nID ([-\d.]+)\r?\nIQ ([-\d.]+)', buf)
        if m: return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None

def cap_run(trigger):
    send('CAP', 0.4)                      # arm
    s.reset_input_buffer()
    s.write((trigger + '\n').encode())
    t0 = time.time(); buf = ''
    while time.time() - t0 < 8.0:
        buf += s.read(65536).decode('ascii', 'replace')
        if 'CAP END' in buf: break
    rows = []
    for m in re.finditer(r'^C\s+(\d+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)',
                         buf, re.M):
        rows.append([float(x) for x in m.groups()[1:]])
    return rows

def analyze(tag, rows, i_ref, i_meas):
    if len(rows) < 50:
        print('  %s: 数据不足 (%d 点)' % (tag, len(rows))); return
    y = [r[i_meas] for r in rows]
    y0 = sum(y[:3]) / 3.0
    yss = sum(y[-50:]) / 50.0
    step = yss - y0
    t10 = t90 = None
    for k, v in enumerate(y):
        if step >= 0:
            if t10 is None and v >= y0 + 0.1 * step: t10 = k
            if t90 is None and v >= y0 + 0.9 * step: t90 = k
        else:
            if t10 is None and v <= y0 + 0.1 * step: t10 = k
            if t90 is None and v <= y0 + 0.9 * step: t90 = k
    peak = max(y) if step >= 0 else min(y)
    os_pct = (peak - yss) / abs(step) * 100 if step else 0.0
    sd = (sum((v - yss) ** 2 for v in y[-50:]) / 50) ** 0.5
    rt = ('%.2f ms' % ((t90 - t10) * DT * 1000)) if (t10 is not None and t90 is not None) else 'n/a'
    print('  %-12s n=%3d ref=%.3f  初值%+.4f 终值%+.4f  超调%+.1f%%  上升(10-90 %%)%s  稳态波动±%.4f'
          % (tag, len(rows), rows[-1][i_ref], y0, yss, os_pct, rt, sd))
    print('     起始5点: ' + ' '.join('%+.4f' % v for v in y[:5]))
    print('     结束5点: ' + ' '.join('%+.4f' % v for v in y[-5:]))

print('== ALIGN ==');            print(send('ALIGN', 1.5))
print('== 参数 ==')
send('PID_D 6 20000', 0.3); send('PID_Q 6 20000', 0.3); send('ID 0.0', 0.2); send('IQ 0.0', 0.2)
print('== RUN ==');              print(send('RUN', 1.0))
st0 = status(); print('RUN 后 STATUS:', st0)

print('\n== 抓取 1：IQ 0 -> 0.10 ==')
r1 = cap_run('IQ 0.10'); analyze('IQ 0->0.10', r1, 1, 3)
st1 = status(); print('  抓取后 STATUS:', st1)

print('\n== 抓取 2：IQ 0.10 -> 0 ==')
r2 = cap_run('IQ 0.00'); analyze('IQ .1->0', r2, 1, 3)

print('\n== 抓取 3：ID 0 -> 0.10 ==')
r3 = cap_run('ID 0.10'); analyze('ID 0->0.10', r3, 0, 2)

print('\n== 还原并 STOP ==')
send('ID 0.0', 0.2); send('IQ 0.0', 0.2)
print(send('STOP', 0.5))
st2 = status(); print('STOP 后 STATUS:', st2)
s.close(); print('CLOSED_OK')
