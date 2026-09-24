"""锁转子后的 q 轴闭环验证。
要点：不做 ALIGN（转子锁住时对齐会得到错误零位）；用 STATUS 核对 IQ_REF 是否真的生效；
     抓取 [id_ref,id,vd,iq,vq,ia,ib]；末尾 STOP + VOFA OFF。"""
import re, statistics as st, struct, time, serial

s = serial.Serial('COM7', 115200, timeout=0.2)
TAIL = b'\x00\x00\x80\x7f'
NAMES = ['id_ref','id','vd','iq','vq','ia','ib']

def send(cmd, wait=0.45):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(9000).decode('ascii', 'replace')

def status(timeout=0.8):
    s.reset_input_buffer(); s.write(b'STATUS\n')
    t0 = time.time(); buf = ''
    while time.time() - t0 < timeout:
        buf += s.read(4096).decode('ascii', 'replace')
        m = re.search(r'ANGLE ([-\d.]+)\r?\nID ([-\d.]+)\r?\nIQ ([-\d.]+)\r?\nID_REF ([-\d.]+)\r?\nIQ_REF ([-\d.]+)', buf)
        if m:
            return dict(ang=float(m.group(1)), id=float(m.group(2)), iq=float(m.group(3)),
                        id_ref=float(m.group(4)), iq_ref=float(m.group(5)))
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

def seg(cmd, secs=1.6):
    print('  -- %s' % cmd)
    print('     ' + send(cmd, 0.3).replace('\n', ' | ')[:120])
    time.sleep(0.3)
    st0 = status(); fr = frames(secs); st1 = status()
    if not fr: return None
    cols = list(zip(*fr)); d = {n: cols[k] for k, n in enumerate(NAMES)}
    return dict(n=len(fr), d=d, s0=st0, s1=st1,
                dang=(st1['ang']-st0['ang']) if (st0 and st1) else float('nan'))

print('== VOFA ON =='); print(send('VOFA ON', 0.5))
print('== CLEAR(若有故障) =='); print(send('CLEAR', 0.5))
print('== RUN =='); print(send('RUN', 1.0))
print('== 参数 =='); print(send('PID_D 6 20000', 0.4)); print(send('PID_Q 6 20000', 0.4))

print('\n== 基线 =='); b = seg('IQ 0.00')
if b: print('   n=%d id=%+.4f(±%.4f) iq=%+.4f(±%.4f) vd=%+.3f vq=%+.3f 角度漂移%+.4f'
            % (b['n'], st.mean(b['d']['id']), st.pstdev(b['d']['id']), st.mean(b['d']['iq']),
               st.pstdev(b['d']['iq']), st.mean(b['d']['vd']), st.mean(b['d']['vq']), b['dang']))

print('\n== d 轴参考 ==')
d1 = seg('ID 0.10'); d2 = seg('ID 0.20')
print('\n== q 轴（锁转子）==')
q0 = seg('ID 0.00')
q1 = seg('IQ 0.05'); q2 = seg('IQ 0.10'); q3 = seg('IQ 0.20')
q4 = seg('IQ 0.00')

print('\n== 汇总 ==')
def rep(tag, r, key, tgt):
    if not r: print('  %-8s 无数据' % tag); return
    d = r['d']; got = st.mean(d[key])
    print('  %-10s n=%3d 目标%.2f -> 实测 %+.4f (误差%+.4f) 波动±%.4f | vd=%+.3f vq=%+.3f | IQ_REF回读=%.3f 角度漂移%+.4f'
          % (tag, r['n'], tgt, got, got - tgt, st.pstdev(d[key]),
             st.mean(d['vd']), st.mean(d['vq']),
             r['s1']['iq_ref'] if r['s1'] else float('nan'), r['dang']))
rep('ID=0.10', d1, 'id', 0.10); rep('ID=0.20', d2, 'id', 0.20)
rep('IQ=0.05', q1, 'iq', 0.05); rep('IQ=0.10', q2, 'iq', 0.10); rep('IQ=0.20', q3, 'iq', 0.20)
rep('IQ=0.00', q4, 'iq', 0.00)

print('\n== 阶跃(coarse: 253Hz) IQ 0->0.10 前 20 帧 ==')
r = seg('IQ 0.00', 0.3); fr = frames(0.5)
send('IQ 0.10', 0.02); fr2 = frames(1.0)
for k in range(0, min(20, len(fr2))):
    print('   t=%5.1fms  id_ref=%.3f id=%+.4f iq=%+.4f vd=%+.3f vq=%+.3f'
          % (k * 1000.0 * len(fr2) and k * (1000.0 / max(1, len(fr2)) * len(fr2) / max(1, len(fr2))) * 0 + k * (1000.0 / 253.0),
             fr2[k][0], fr2[k][1], fr2[k][3], fr2[k][2], fr2[k][4]))

print('\n== STOP =='); print(send('STOP', 0.5))
print('== VOFA OFF =='); print(send('VOFA OFF', 0.4))
s.close(); print('CLOSED_OK')
