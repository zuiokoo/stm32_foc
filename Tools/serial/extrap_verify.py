"""新固件验证：① 静止时电流环没退化  ② 转子转动时电角度外推是否把波动压下来
（对照：改之前 转子自由 IQ0.5 时 id std 40 LSB / iq std 31 LSB，频谱有 200/400/600Hz 线谱）
"""
import serial, time, math
import numpy as np

PORT = 'COM7'
LSB = (3.3 / 4095.0) / (0.005 * 50.0)
R = 3.55
FS = 20000.0
DROP = 100


class Board:
    def __init__(self):
        self.s = serial.Serial(PORT, 115200, timeout=0.05)

    def send(self, c, w=0.05):
        self.s.write((c + '\r\n').encode()); time.sleep(w)

    def lines(self, dur):
        out, t0 = [], time.time()
        while time.time() - t0 < dur:
            ln = self.s.readline()
            if ln:
                out.append(ln.decode('ascii', 'replace').rstrip())
        return out

    def waitfor(self, sub, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ln = self.s.readline()
            if ln and sub in ln.decode('ascii', 'replace'):
                return True
        return False

    def status(self):
        self.send('STATUS')
        d, t0 = {}, time.time()
        while time.time() - t0 < 1.0:
            ln = self.s.readline()
            if not ln:
                continue
            t = ln.decode('ascii', 'replace').strip()
            for k in ('ANGLE', 'SPD', 'SPD_REF', 'IQ_REF', 'ID_REF', 'EXTRACLIP', 'SPD_EN'):
                if t.startswith(k + ' '):
                    try:
                        d[k] = float(t.split()[1])
                    except ValueError:
                        pass
        return d

    def capture(self, cmd):
        self.send('CAP')
        if not self.waitfor('CAP ARMED', 2.0):
            return None
        self.send(cmd)
        if not self.waitfor('CAP BEGIN', 6.0):
            print('    CAP 未触发'); return None
        data, t0 = [], time.time()
        while time.time() - t0 < 8.0 and len(data) < 400:
            ln = self.s.readline()
            if not ln:
                continue
            t = ln.decode('ascii', 'replace').strip()
            if t.startswith('CAP END'):
                break
            if t.startswith('C '):
                p = t.split()
                if len(p) == 8:
                    data.append([float(x) for x in p[2:8]])
        return data


def rep(tag, data):
    if not data:
        print('  %s: 无数据' % tag); return
    a = np.array(data[DROP:])
    idc, iqc, vd, vq = a[:, 2], a[:, 3], a[:, 4], a[:, 5]
    y = iqc - iqc.mean()
    w = np.hanning(len(y))
    sp = np.abs(np.fft.rfft(y * w)) * 2 / w.sum()
    fr = np.fft.rfftfreq(len(y), 1 / FS)
    def bm(lo, hi):
        m = (fr > lo) & (fr < hi)
        return sp[m].max() if m.any() else 0
    print('  %-22s iq std %.2f LSB (峰峰 %.1f) | id std %.2f LSB | 反电势%+.3fV | 谱: 200Hz %.4f 400Hz %.4f 300-3k %.4f'
          % (tag, iqc.std() / LSB, np.ptp(iqc) / LSB, idc.std() / LSB,
             vq.mean() - R * iqc.mean(), bm(180, 220), bm(380, 420), bm(300, 3000)))


b = Board()
print('OPEN_OK')
print('--- 上电自检（被动读 2.5s）---')
for l in b.lines(2.5):
    print('   ', l)

b.send('VOFA OFF'); b.lines(0.3)
print('--- 1) 静止：电流环有没有退化（期望 iq std ~1.5 LSB，无 200/400Hz 线）---')
b.send('STOP'); b.lines(0.3)
b.send('CLEAR'); b.lines(0.3)
b.send('RUN'); print('   RUN:', b.waitfor('RUN: pwm started', 3.0))
b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)
print('   STATUS:', {k: round(v, 3) for k, v in b.status().items()})
rep('ID 0.5 静止', b.capture('ID 0.5'))
b.lines(0.3)

print('--- 2) 转子自由转动（手动 IQ 0.2，转子会转起来）---')
b.send('ID 0.0'); b.send('IQ 0.2'); b.lines(2.0)
print('   转动中 STATUS:', {k: round(v, 3) for k, v in b.status().items()})
rep('IQ 0.2 转动中', b.capture('IQ 0.2'))
print('   转速:', {k: round(v, 3) for k, v in b.status().items()})

print('--- 3) 速度环：SPD 2 (约 19 rpm) ---')
b.send('IQ 0.0'); b.lines(0.5)
b.send('PID_S 0.02 0.1')
print('   STATUS:', {k: round(v, 3) for k, v in b.status().items()})

print('--- 停止 ---')
b.send('SPD_OFF'); b.lines(0.3)
b.send('IQ 0.0'); b.lines(0.2)
b.send('STOP'); b.lines(0.8)
b.s.close()
