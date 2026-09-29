"""判定实验：2kHz 振荡是"环路自激"还是"测量链噪声/干扰"？
做法：把 PI 增益置 0（环路不动作，占空比固定 50%，电机无电流），看测到的
       id/iq 还有没有 2kHz —— 有 = 测量链的干扰；没有 = 环路把它放大出来的。
再对比半增益，看幅度随增益怎么变。
"""
import serial, time, math
import numpy as np

PORT = 'COM7'
FS = 20000.0
DROP = 100


class Board:
    def __init__(self):
        self.s = serial.Serial(PORT, 115200, timeout=0.05)

    def send(self, cmd, wait=0.05):
        self.s.write((cmd + '\r\n').encode())
        time.sleep(wait)

    def lines(self, dur):
        out, t0 = [], time.time()
        while time.time() - t0 < dur:
            ln = self.s.readline()
            if ln:
                out.append(ln.decode('ascii', 'replace').rstrip())
        return out

    def waitfor(self, sub, timeout=6.0):
        t0 = time.time()
        seen = []
        while time.time() - t0 < timeout:
            ln = self.s.readline()
            if ln:
                t = ln.decode('ascii', 'replace').rstrip()
                seen.append(t)
                if sub in t:
                    return True, seen
        return False, seen

    def capture(self, cmd):
        self.send('CAP')
        if not self.waitfor('CAP ARMED', 2.0)[0]:
            return None
        self.send(cmd)
        if not self.waitfor('CAP BEGIN', 6.0)[0]:
            return None
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


def report(tag, data):
    if not data or len(data) < 200:
        print('  %s: 数据不足' % tag)
        return None
    a = np.array(data[DROP:])
    idr, iqr, idc, iqc, vd, vq = [a[:, i] for i in range(6)]
    print('  %-26s iq: 均值%+.4f std %.4f 峰峰 %.4f | id: std %.4f 峰峰 %.4f | vq: std %.4f 峰峰 %.4f'
          % (tag, np.mean(iqc), np.std(iqc), np.ptp(iqc), np.std(idc), np.ptp(idc),
             np.std(vq), np.ptp(vq)))
    y = iqc - np.mean(iqc)
    w = np.hanning(len(y))
    sp = np.abs(np.fft.rfft(y * w)) * 2.0 / w.sum()
    fr = np.fft.rfftfreq(len(y), 1.0 / FS)
    idx = np.argsort(sp[1:])[::-1][:3] + 1
    band = []
    for lo, hi in ((0, 300), (300, 1000), (1000, 3000), (3000, 10000)):
        m = (fr >= lo) & (fr < hi)
        band.append('%d-%dHz:%.4f' % (lo, hi, sp[m].max() if m.any() else 0))
    print('      iq 谱峰: %s | 分带: %s'
          % (', '.join('%.0fHz:%.4f' % (fr[i], sp[i]) for i in idx), '  '.join(band)))
    d = np.diff(iqc)
    print('      相邻差std/白噪声理论 = %.2f  (<<1 低频/振荡, ≈1 白噪声)   '
          'vq×iq 相关 lag0 = %+.2f' % (np.std(d) / (math.sqrt(2) * np.std(iqc) + 1e-12),
                                       np.corrcoef(vq, iqc)[0, 1]))
    return np.std(iqc), np.ptp(iqc)


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.send('STOP'); b.lines(1.0)
b.send('CLEAR'); b.lines(0.3)
b.send('RUN')
ok, seen = b.waitfor('RUN: pwm started', 3.0)
print('RUN:', ok, [x for x in seen if 'RUN' in x])
b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)

print('\n=== A: 环路关闭（PID 增益全 0，占空比固定 50%，电机无电流）===')
b.send('PID_D 0 0'); b.send('PID_Q 0 0'); b.lines(0.3)
b.send('IQ 0.5'); b.lines(0.3)
A = report('增益 0（纯测量链）', b.capture('IQ 0.5'))

print('\n=== B: 标定增益 kp=2 ki=10000 ===')
b.send('PID_D 2 10000'); b.send('PID_Q 2 10000'); b.lines(0.3)
B = report('kp=2 ki=10000', b.capture('IQ 0.5'))
b.lines(0.3)

print('\n=== C: 半增益 kp=1 ki=5000 ===')
b.send('PID_D 1 5000'); b.send('PID_Q 1 5000'); b.lines(0.3)
C = report('kp=1 ki=5000', b.capture('IQ 0.5'))
b.lines(0.3)

print('\n=== D: 只降积分 ki=2000 (kp 保持 2) ===')
b.send('PID_D 2 2000'); b.send('PID_Q 2 2000'); b.lines(0.3)
D = report('kp=2 ki=2000', b.capture('IQ 0.5'))
b.lines(0.3)

print('\n=== 恢复 kp=2 ki=10000 并停机 ===')
b.send('PID_D 2 10000'); b.send('PID_Q 2 10000'); b.lines(0.2)
b.send('IQ 0.0'); b.lines(0.2)
b.send('STOP'); b.lines(0.6)
if A and B and C:
    print('\n小结: 峰峰  A(无环路) %.4f  ->  B(kp2,ki10000) %.4f  ->  C(kp1,ki5000) %.4f'
          % (A[1], B[1], C[1]))
    print('      std   A %.4f  ->  B %.4f  ->  C %.4f' % (A[0], B[0], C[0]))
b.s.close()
