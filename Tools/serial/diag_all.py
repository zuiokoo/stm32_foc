"""诊断：逐通道打印全部 7 路（含 ia/ib）+ 与 STATUS 交叉核对，确认数据可信后再看环。
注意：不做 ALIGN（转子锁住，避免把零位搞坏）。"""
import re, statistics as st, struct, time, serial

s = serial.Serial('COM7', 115200, timeout=0.2)
TAIL = b'\x00\x00\x80\x7f'
NAMES = ['id_ref','id','vd','iq','vq','ia','ib']

def send(cmd, wait=0.45):
    s.reset_input_buffer(); s.write((cmd + '\n').encode()); time.sleep(wait)
    return s.read(9000).decode('ascii', 'replace')

def status(timeout=1.0):
    s.reset_input_buffer(); s.write(b'STATUS\n')
    t0 = time.time(); buf = ''
    while time.time() - t0 < timeout:
        buf += s.read(4096).decode('ascii', 'replace')
        m = re.search(r'ANGLE ([-\d.]+)\r?\nID ([-\d.]+)\r?\nIQ ([-\d.]+)\r?\nID_REF ([-\d.]+)\r?\nIQ_REF ([-\d.]+)', buf)
        if m: return dict(ang=float(m.group(1)), id=float(m.group(2)), iq=float(m.group(3)),
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

def probe(tag, secs=1.2, show_frames=2):
    print('\n>>> %s' % tag)
    stt = status()
    fr = frames(secs)
    if not fr:
        print('   no frames'); return
    cols = list(zip(*fr))
    print('   STATUS: ang=%.3f id=%.4f iq=%.4f id_ref=%.3f iq_ref=%.3f' %
          (stt['ang'], stt['id'], stt['iq'], stt['id_ref'], stt['iq_ref']) if stt else '   STATUS: None')
    for k, n in enumerate(NAMES):
        v = cols[k]
        print('   %-7s mean=%+.4f  sd=%.4f  min=%+.4f  max=%+.4f' %
              (n, st.mean(v), st.pstdev(v), min(v), max(v)))
    for i in range(min(show_frames, len(fr))):
        print('   raw[%d] '%i + ' '.join('%s=%+.3f' % (n, fr[i][k]) for k, n in enumerate(NAMES)))

print('== VOFA ON =='); print(send('VOFA ON', 0.5))
print('== 参数回执 ==')
print('  PID_D:', send('PID_D 6 20000', 0.4).strip().replace('\r', ' ').replace('\n', ' ')[:60])
print('  PID_Q:', send('PID_Q 6 20000', 0.4).strip().replace('\r', ' ').replace('\n', ' ')[:60])
print('  ID 0.0:', send('ID 0.0', 0.3).strip().replace('\r', ' ').replace('\n', ' ')[:60])
print('  IQ 0.0:', send('IQ 0.0', 0.3).strip().replace('\r', ' ').replace('\n', ' ')[:60])
print('== RUN =='); print(send('RUN', 1.0))

probe('静止 / 两轴给定 0')
send('ID 0.10', 0.3); time.sleep(0.5); probe('ID = 0.10（d 轴，期望 id=+0.10 vd≈+0.35 vq≈0）')
send('ID 0.00', 0.3); send('IQ 0.10', 0.3); time.sleep(0.5); probe('IQ = 0.10（期望 iq=+0.10 vq≈+0.35 vd≈0）')
send('IQ 0.00', 0.3); send('ID 0.00', 0.3); time.sleep(0.5)

print('\n== STOP =='); print(send('STOP', 0.5))
print('== VOFA OFF =='); print(send('VOFA OFF', 0.4))
s.close(); print('CLOSED_OK')
