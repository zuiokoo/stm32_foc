"""抓取并存盘 + 转子状态判据。
转子是否真的固定，用两个物理量判断：
  1) 隐含反电势 = vq均值 - R*iq均值  （固定/静止时 ≈0；转动时明显不为 0）
  2) 频谱里 200/400/600Hz 的谐波（编码器 5ms 更新一次，转子一动就会激发这些线谱）
"""
import serial, time, math, os
import numpy as np

PORT = 'COM7'
FS = 20000.0
DROP = 100
R_PHASE = 3.53
OUT = os.path.dirname(os.path.abspath(__file__))


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

    def angle(self):
        self.send('STATUS')
        t0 = time.time()
        while time.time() - t0 < 1.0:
            ln = self.s.readline()
            if ln:
                t = ln.decode('ascii', 'replace').strip()
                if t.startswith('ANGLE'):
                    return float(t.split()[1])
        return None

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


def report(tag, data, fname):
    if not data or len(data) < 200:
        print('  %s: 数据不足' % tag); return
    a = np.array(data[DROP:])
    with open(os.path.join(OUT, fname), 'w') as f:
        f.write('id_ref,iq_ref,id,iq,vd,vq\n')
        for r in a:
            f.write('%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % tuple(r))
    idr, iqr, idc, iqc, vd, vq = [a[:, i] for i in range(6)]
    emf = np.mean(vq) - R_PHASE * np.mean(iqc)
    print('  %-22s 给定 iq_ref=%+.3f id_ref=%+.3f' % (tag, iqr[-1], idr[-1]))
    print('      iq 均值%+.4f std %.4f 峰峰 %.4f (%.1f%%)  | id 均值%+.4f std %.4f 峰峰 %.4f'
          % (np.mean(iqc), np.std(iqc), np.ptp(iqc), 100 * np.ptp(iqc) / 0.5,
             np.mean(idc), np.std(idc), np.ptp(idc)))
    print('      vq 均值%+.4f std %.4f | vd 均值%+.4f std %.4f | 隐含反电势 %+.4f V %s'
          % (np.mean(vq), np.std(vq), np.mean(vd), np.std(vd), emf,
             '(≈0 -> 转子没动)' if abs(emf) < 0.15 else '(明显非 0 -> 转子在动!)'))
    y = iqc - np.mean(iqc)
    w = np.hanning(len(y))
    sp = np.abs(np.fft.rfft(y * w)) * 2.0 / w.sum()
    fr = np.fft.rfftfreq(len(y), 1.0 / FS)
    def bandmax(lo, hi):
        m = (fr >= lo) & (fr < hi)
        return sp[m].max() if m.any() else 0
    print('      谱: 200Hz线%.4f 400Hz线%.4f | 带 0-300:%.4f 300-3k:%.4f 3k-10k:%.4f'
          % (bandmax(180, 220), bandmax(380, 420), bandmax(0, 300), bandmax(300, 3000), bandmax(3000, 10000)))
    print('      存盘 -> %s' % fname)


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.send('STOP'); b.lines(1.0)
b.send('CLEAR'); b.lines(0.3)
b.send('RUN')
print('RUN:', b.waitfor('RUN: pwm started', 3.0)[0])
b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)
a0 = b.angle(); print('起始电角度 %.3f rad' % (a0 if a0 else -1))

print('\n=== 1) 环路关闭（增益 0）：纯测量链底噪 ===')
b.send('PID_D 0 0'); b.send('PID_Q 0 0'); b.lines(0.3)
report('增益0 纯测量链', b.capture('IQ 0.5'), 'cap_gain0.csv')
b.lines(0.3)

print('\n=== 2) 恢复增益，ID 0.5（d 轴，理论无转矩）===')
b.send('PID_D 2 10000'); b.send('PID_Q 2 10000'); b.lines(0.3)
report('ID 0.5 (d轴)', b.capture('ID 0.5'), 'cap_id05.csv')
a1 = b.angle(); print('      ID0.5 后电角度 %.3f rad (变化 %+.3f)'
                      % (a1 if a1 else -1, (a1 - a0) if (a1 and a0) else 0))
b.lines(0.3)

print('\n=== 3) IQ 0.5（q 轴，你的工况）===')
b.send('ID 0.0'); b.lines(0.2)
report('IQ 0.5 (q轴)', b.capture('IQ 0.5'), 'cap_iq05.csv')
a2 = b.angle(); print('      IQ0.5 后电角度 %.3f rad (变化 %+.3f)' % (a2 if a2 else -1, (a2 - a1) if (a2 and a1) else 0))
b.lines(0.3)

print('\n=== 4) 零给定 + 环路工作：环路自身的本底 ===')
b.send('IQ 0.0'); b.lines(0.3)
report('IQ 0 (环路工作)', b.capture('IQ 0.0'), 'cap_iq0.csv')

print('\n=== 恢复并停机 ===')
b.send('ID 0.0'); b.send('IQ 0.0')
b.send('STOP'); b.lines(0.6)
b.s.close()
