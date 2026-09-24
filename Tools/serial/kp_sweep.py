"""kp 扫描：保持 ki/kp 比例不变(=3333，即极零抵消)，看不同 kp 下的阶跃超调。
每档用 CAP 抓一次 IQ 0->0.10 的阶跃。"""
import re, time, serial

s = serial.Serial('COM7', 115200, timeout=0.2)
DT = 50e-6

def send(cmd, wait=0.4):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(9000).decode('ascii', 'replace')

def cap_run(trigger):
    send('CAP', 0.3)
    s.reset_input_buffer(); s.write((trigger + '\n').encode())
    t0 = time.time(); buf = ''
    while time.time() - t0 < 8.0:
        buf += s.read(65536).decode('ascii', 'replace')
        if 'CAP END' in buf: break
    rows = []
    for m in re.finditer(r'^C\s+(\d+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', buf, re.M):
        rows.append([float(x) for x in m.groups()[1:]])
    return rows

def metrics(rows, i_ref=1, i_meas=3):
    """返回 超调%、上升ms、稳态误差、稳态波动"""
    if len(rows) < 50: return None
    y = [r[i_meas] for r in rows]
    y0 = sum(y[:3]) / 3.0; yss = sum(y[-50:]) / 50.0
    step = yss - y0
    t10 = t90 = None
    for k, v in enumerate(y):
        if v >= y0 + 0.1 * step and t10 is None: t10 = k
        if v >= y0 + 0.9 * step and t90 is None: t90 = k
    peak = max(y)
    os_pct = (peak - yss) / step * 100 if step else 0.0
    sd = (sum((v - yss) ** 2 for v in y[-50:]) / 50) ** 0.5
    rt = (t90 - t10) * DT * 1000 if (t10 is not None and t90 is not None) else float('nan')
    ref = rows[-1][i_ref]
    return dict(os=os_pct, rt=rt, yss=yss, err=yss - ref, sd=sd,
                vq=sum(r[5] for r in rows[-50:]) / 50.0, vd=sum(r[4] for r in rows[-50:]) / 50.0)

send('ID 0.0', 0.2); send('IQ 0.0', 0.2)
for kp in [6.0, 3.0, 2.0, 1.5]:
    ki = kp * 3333.0
    print('== kp=%.1f ki=%.0f ==' % (kp, ki))
    print('  ' + send('PID_D %.1f %.0f' % (kp, ki), 0.3).strip().replace('\n', ' ')[:40],
          '|', send('PID_Q %.1f %.0f' % (kp, ki), 0.3).strip().replace('\n', ' ')[:40])
    send('IQ 0.00', 0.3); time.sleep(0.4)          # 回到 0
    rows = cap_run('IQ 0.10')
    m = metrics(rows)
    if m:
        print('  超调 %+6.1f%%   上升 %5.2f ms   稳态 %+.4f (误差%+.4f, 波动±%.4f)   vq=%+.3f vd=%+.3f'
              % (m['os'], m['rt'], m['yss'], m['err'], m['sd'], m['vq'], m['vd']))
    else:
        print('  数据不足')
    send('IQ 0.00', 0.3); time.sleep(0.3)

# 还原一组保守参数并停止
send('PID_D 2.0 6700', 0.3); send('PID_Q 2.0 6700', 0.3)
send('IQ 0.0', 0.2); send('ID 0.0', 0.2)
print(send('STOP', 0.5))
s.close(); print('CLOSED_OK')
