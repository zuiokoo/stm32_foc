"""速度环第一轮实测：
 A) 低电流转动（IQ 0.1，电压有余量）—— 顺便验电角度外推：转动时电流带应该只剩几个 LSB
 B) 速度环阶跃（SPD 10 / 30）—— 看跟随、超调、稳态误差
自带发散保护（|spd| 远大于给定且 iq 顶限幅 -> 立刻 SPD_OFF）
"""
import serial, time, math, sys
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

    def waitfor(self, sub, timeout=8.0):
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
            for k in ('ANGLE', 'SPD', 'SPD_REF', 'IQ', 'IQ_REF', 'EXTRACLIP', 'SPD_EN'):
                if t.startswith(k + ' '):
                    try:
                        d[k] = float(t.split()[1])
                    except ValueError:
                        pass
        return d

    def cap(self, cmd):
        self.send('CAP')
        if not self.waitfor('CAP ARMED', 2.0):
            print('   CAP 未 armed'); return None
        self.send(cmd)
        if not self.waitfor('CAP BEGIN', 6.0):
            print('   CAP 未触发'); return None
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

    def caps(self, ref):
        self.send('CAPS')
        if not self.waitfor('CAPS ARMED', 2.0):
            print('   CAPS 未 armed'); return None
        self.send('SPD %.4f' % ref)
        if not self.waitfor('CAPS BEGIN', 8.0):
            print('   CAPS 未触发'); return None
        data, t0 = [], time.time()
        while time.time() - t0 < 10.0 and len(data) < 400:
            ln = self.s.readline()
            if not ln:
                continue
            t = ln.decode('ascii', 'replace').strip()
            if t.startswith('CAPS END'):
                break
            if t.startswith('S '):
                p = t.split()
                if len(p) == 6:
                    data.append([float(x) for x in p[2:6]])
                    if len(data) % 50 == 0 and len(data) > 60:
                        if abs(data[-1][1]) > 5 * abs(ref) + 30 and abs(data[-1][3]) > 0.55:
                            self.send('SPD_OFF'); self.send('STOP')
                            print('   !! 发散 spd=%.1f iq=%.2f -> SPD_OFF' % (data[-1][1], data[-1][3]))
                            return None
        return data


def cap_rep(tag, data):
    if not data:
        print('  %s 无数据' % tag); return
    a = np.array(data[DROP:])
    idc, iqc, vd, vq = a[:, 2], a[:, 3], a[:, 4], a[:, 5]
    y = iqc - iqc.mean(); w = np.hanning(len(y))
    sp = np.abs(np.fft.rfft(y * w)) * 2 / w.sum(); fr = np.fft.rfftfreq(len(y), 1 / FS)
    bm = lambda lo, hi: (sp[(fr > lo) & (fr < hi)].max() if ((fr > lo) & (fr < hi)).any() else 0)
    print('  %-16s iq std %.2f LSB 峰峰 %.1f | id std %.2f LSB | vq均%+.3f vd均%+.3f | 谱 200Hz %.4f 400Hz %.4f 300-3k %.4f'
          % (tag, iqc.std() / LSB, np.ptp(iqc) / LSB, idc.std() / LSB, vq.mean(), vd.mean(),
             bm(180, 220), bm(380, 420), bm(300, 3000)))
    print('                 参照: 静止时 id/iq std ≈1.2~1.6 LSB, 谱线 ≈0.0005')


def spd_rep(tag, d, ref):
    if not d or len(d) < 100:
        print('  %s 数据不足' % tag); return
    a = np.array(d)
    spr, sp, iqr, iq = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    ss = sp[-100:]
    print('  %s 给定 %.1f rad/s (%.0f rpm)' % (tag, ref, ref * 60 / 2 / math.pi))
    print('    稳态 spd %.2f (%.1f rpm) 误差%+.1f%%  标准差 %.2f  iq 均%.3f (峰 %.2f/%.2f)'
          % (ss.mean(), ss.mean() * 60 / 2 / math.pi, 100 * (ss.mean() - ref) / ref, ss.std(), iq[-100:].mean(), iq.max(), iq.min()))
    pk = sp.max() if ref > 0 else sp.min()
    print('    峰值 %.2f (超调 %+.0f%%)' % (pk, 100 * (pk - ref) / ref))
    print('    前15点 spd: ' + ' '.join('%.1f' % v for v in sp[:15]))


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.lines(0.8)
print('停机归零 ...')
b.send('STOP'); b.lines(0.3)
b.send('SPD_OFF'); b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)
b.send('CLEAR'); b.lines(0.3)
print('RUN:', b.waitfor('RUN: pwm started', 3.0))
st = b.status(); print('起始状态:', {k: round(v, 3) for k, v in st.items()})

print('\n=== A) 低电流自由转动：IQ 0.1（验电角度外推）===')
b.send('IQ 0.1'); b.lines(2.5)
st = b.status(); print('   转动中: SPD=%.1f rad/s (%.0f rpm) IQ=%.3f EXTRACLIP=%d'
                       % (st.get('SPD', 0), st.get('SPD', 0) * 60 / 2 / math.pi, st.get('IQ', 0), st.get('EXTRACLIP', 0)))
cap_rep('IQ0.1 转动', b.cap('IQ 0.1'))
if abs(st.get('SPD', 0)) < 5:
    print('   !! 转速≈0：转子可能被夹住了，速度环测不出效果，请松开后再跑')
b.lines(0.5)

print('\n=== B) 速度环阶跃 ===')
b.send('IQ 0.0'); b.lines(0.5)
b.send('PID_S 0.02 0.1')
for ref in (10.0, 30.0):
    d = b.caps(ref)
    spd_rep('SPD %.0f' % ref, d, ref)
    print('    STATUS:', {k: round(v, 2) for k, v in b.status().items()})
    if d is None:
        break
    b.lines(0.3)

print('\n=== 收尾 ===')
b.send('SPD_OFF'); b.lines(0.3)
b.send('IQ 0.0'); b.lines(0.2)
b.send('STOP'); b.lines(0.8)
b.s.close()
