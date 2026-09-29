"""MA8 下的精确对比：环路关（纯测量链）vs 环路工作。
只看 std（LSB），因为 14 帧的峰峰统计不可靠，CAP 有 300 点。
"""
import serial, time, math
import numpy as np

PORT = 'COM7'
LSB = (3.3 / 4095.0) / (0.005 * 50.0)
R_PHASE = 3.55
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
        t0 = time.time(); seen = []
        while time.time() - t0 < timeout:
            ln = self.s.readline()
            if ln:
                t = ln.decode('ascii', 'replace').rstrip(); seen.append(t)
                if sub in t:
                    return True, seen
        return False, seen

    def capture(self, cmd):
        self.send('CAP')
        if not self.waitfor('CAP ARMED', 2.0)[0]:
            return None
        self.send(cmd)
        if not self.waitfor('CAP BEGIN', 6.0)[0]:
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
    a = np.array(data[DROP:])
    idc, iqc, vd, vq = a[:, 2], a[:, 3], a[:, 4], a[:, 5]
    emf = np.mean(vq) - R_PHASE * np.mean(iqc)
    y = iqc - np.mean(iqc)
    w = np.hanning(len(y))
    sp = np.abs(np.fft.rfft(y * w)) * 2 / w.sum()
    fr = np.fft.rfftfreq(len(y), 1 / 20000.0)
    b3k = sp[(fr > 300) & (fr < 3000)].max()
    print('  %-20s iq std %.2f LSB (%.4f A)  峰峰 %.1f LSB | id std %.2f LSB | vq std %.4f | 反电势%+.3fV | 300-3kHz峰 %.4f'
          % (tag, np.std(iqc) / LSB, np.std(iqc), np.ptp(iqc) / LSB, np.std(idc) / LSB, np.std(vq), emf, b3k))


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.send('STOP'); b.lines(1.0)
b.send('CLEAR'); b.lines(0.3)
b.send('RUN'); print('RUN:', b.waitfor('RUN: pwm started', 3.0)[0])
b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)

print('\n--- 环路关（增益 0）：纯测量链噪声，MA8 生效后 ---')
b.send('PID_D 0 0'); b.send('PID_Q 0 0'); b.lines(0.3)
b.send('IQ 0.5'); b.lines(0.3)
rep('增益0 纯测量链', b.capture('IQ 0.5'))
b.lines(0.3)

print('--- 环路工作，零给定 ---')
b.send('PID_D 2 10000'); b.send('PID_Q 2 10000'); b.lines(0.3)
b.send('IQ 0.0'); b.lines(0.3)
rep('IQ 0 (环路工作)', b.capture('IQ 0.0'))
b.lines(0.3)

print('--- 环路工作，IQ 0.5 ---')
rep('IQ 0.5', b.capture('IQ 0.5'))

print('\n--- 停机 ---')
b.send('IQ 0.0'); b.lines(0.2)
b.send('STOP'); b.lines(0.6)
print('  参考: MA4 时 纯测量链 std 1.47 LSB / 环路工作(IQ0.5) 1.24 LSB')
b.s.close()
