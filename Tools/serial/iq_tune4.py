"""烧写后完整测试：靠解析 VOFA 的 JustFloat 二进制帧拿高速数据（~百 Hz 级）。
抓取布局: [id_ref, id, vd, iq, vq, ia, ib]
安全: id/iq <= 0.2A，末尾 STOP。"""
import re, statistics as st, struct, time, serial

s = serial.Serial('COM7', 115200, timeout=0.2)
TAIL = b'\x00\x00\x80\x7f'

def send(cmd, wait=0.5):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(9000).decode('ascii', 'replace')

def status(timeout=0.8):
    s.reset_input_buffer(); s.write(b'STATUS\n')
    t0 = time.time(); buf = ''
    while time.time() - t0 < timeout:
        buf += s.read(4096).decode('ascii', 'replace')
        m = re.search(r'ANGLE ([-\d.]+)\r?\nID ([-\d.]+)\r?\nIQ ([-\d.]+)', buf)
        if m: return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None

def frames(secs):
    buf = bytearray(); out = []
    t0 = time.time()
    while time.time() - t0 < secs:
        buf += s.read(8192)
        while True:
            i = buf.find(TAIL)
            if i < 28: break
            out.append(struct.unpack('<7f', bytes(buf[i-28:i])))
            del buf[:i+4]
        if len(buf) > 20000: del buf[:-8]
    return out

def run_seg(cmd, secs=2.0):
    send(cmd, 0.3); time.sleep(0.3)
    a0 = status()
    fr = frames(secs)
    a1 = status()
    if not fr: return None
    cols = list(zip(*fr))
    names = ['id_ref','id','vd','iq','vq','ia','ib']
    d = {n: cols[k] for k, n in enumerate(names)}
    age = (time.time() - a0[0]) if False else 0
    return dict(n=len(fr), rate=len(fr)/secs, d=d,
                ang=(a1[0]-a0[0]) if (a0 and a1) else float('nan'),
                id_ref=d['id_ref'][-1], cmd=cmd)

print('== VOFA ON ==');  print(send('VOFA ON', 0.5))
print('== ALIGN ==');    print(send('ALIGN', 1.5))
send('PID_D 6 20000'); send('PID_Q 6 20000'); send('ID 0.0'); send('IQ 0.0')
print('== RUN ==');      print(send('RUN', 1.0))

segs = ['ID 0.05','ID 0.10','ID 0.20','ID 0.00','IQ 0.05','IQ 0.10','IQ 0.20','IQ 0.00']
res = []
for c in segs:
    r = run_seg(c)
    if r:
        d = r['d']
        print('%-8s n=%3d(%4.0fHz)  id_ref=%.3f  id=%+.4f(±%.4f)  vd=%+.3f  iq=%+.4f(±%.4f)  vq=%+.3f  角度漂移%+.3f'
              % (c, r['n'], r['rate'], r['id_ref'], st.mean(d['id']), st.pstdev(d['id']),
                 st.mean(d['vd']), st.mean(d['iq']), st.pstdev(d['iq']), st.mean(d['vq']), r['ang']))
        res.append((c, r))
    else:
        print('%-8s 无数据!' % c)

print('== STOP =='); print(send('STOP', 0.5))
print('== 汇总（电流稳态精度）==')
for c, r in res:
    d = r['d']; key = 'id' if c.startswith('ID') else 'iq'
    tgt = float(c.split()[1])
    got = st.mean(d[key])
    print('  %-8s 目标 %.2f -> 实测 %+.4f  误差 %+.4f  波动 ±%.4f  偏置(末帧) %+.4f'
          % (c, tgt, got, got-tgt, st.pstdev(d[key]), r['id_ref'] if key=='id' else st.mean(d['iq_ref']) if 'iq_ref' in d else 0))
s.close(); print('CLOSED_OK')
