"""速度环第二轮：ki=0（纯 P）扫描 kp。
理由：上一轮发现 kp=0 的纯积分也在震 —— 说明机械阻尼≈0，
      积分作用在纯惯量对象上是双积分器(相位180°)，必震；
      而"无摩擦"意味着维持转速不需要力矩，所以纯 P 也能做到零稳态误差。
"""
import serial, time, math
import numpy as np

PORT = 'COM7'
REF = 30.0
PTS = [(0.0002, 0.0), (0.0005, 0.0), (0.001, 0.0), (0.002, 0.0), (0.005, 0.0), (0.001, 0.005)]


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
            for k in ('SPD', 'IQ'):
                if t.startswith(k + ' '):
                    try:
                        d[k] = float(t.split()[1])
                    except ValueError:
                        pass
        return d

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
                    if len(data) % 25 == 0 and len(data) > 60:
                        if abs(data[-1][1]) > 5 * abs(ref) + 30 and abs(data[-1][3]) > 0.55:
                            self.send('SPD_OFF'); self.send('STOP')
                            print('   !! 发散 -> SPD_OFF')
                            return None
        return data


def judge(d, ref):
    a = np.array(d)
    sp, iq = a[:, 1], a[:, 3]
    ss = sp[-100:]
    # 主振荡频率：对最后 1.5s 做 FFT
    y = sp[-300:] - sp[-300:].mean()
    if len(y) >= 32:
        sp_fft = np.abs(np.fft.rfft(y * np.hanning(len(y))))
        fr = np.fft.rfftfreq(len(y), 0.005)
        f0 = fr[1:][np.argmax(sp_fft[1:])]
    else:
        f0 = 0
    return dict(ov=100 * (sp[:250].max() - ref) / ref,
                err=100 * (ss.mean() - ref) / ref,
                rip=100 * np.std(ss) / ref,
                f=f0, iq=iq[-100:].mean(), iqpk=iq.max())


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.lines(0.6)
b.send('STOP'); b.lines(0.3)
b.send('SPD_OFF'); b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.4)

ok = False
for attempt in range(4):
    b.send('CLEAR'); b.lines(0.4)
    b.send('RUN');   b.lines(0.6)
    b.send('IQ 0.3'); b.lines(1.5)
    st = b.status()
    print('  尝试%d: SPD=%.1f IQ=%.3f' % (attempt + 1, st.get('SPD', 0), st.get('IQ', 0)))
    if abs(st.get('SPD', 0)) > 20:
        ok = True; break
if not ok:
    print('!! RUN 未起来'); b.s.close(); raise SystemExit
print('转子自由、RUNNING ✓')
b.send('IQ 0.0'); b.lines(3.0)

print('\n%-16s %8s %8s %8s %9s %8s %7s' % ('(kp,ki)', '超调%', '稳态误差%', '稳态std%', '振荡Hz', 'iq稳态', 'iq峰'))
best = None
for kp, ki in PTS:
    b.send('SPD_OFF'); b.send('IQ 0.0'); b.lines(0.2)
    b.send('PID_S %g %g' % (kp, ki)); b.lines(0.2)
    b.lines(1.5)
    d = b.caps(REF)
    if d is None:
        print('%-16s 失败' % ('(%.4g,%.3g)' % (kp, ki))); continue
    j = judge(d, REF)
    print('%-16s %+8.1f %+8.1f %8.2f %9.1f %8.3f %7.3f'
          % ('(%.4g,%.3g)' % (kp, ki), j['ov'], j['err'], j['rip'], j['f'], j['iq'], j['iqpk']))
    np.savetxt('spd_%.5g_%.5g.csv' % (kp, ki), np.array(d), delimiter=',',
               header='speed_ref,speed,iq_ref,iq', comments='')
    score = abs(j['err']) + j['rip'] + max(0, j['ov']) * 0.2
    if best is None or score < best[0]:
        best = (score, kp, ki, j)
    b.lines(0.3)

print('\n--- 收尾 ---')
b.send('SPD_OFF'); b.lines(0.2)
b.send('IQ 0.0'); b.lines(0.2)
b.send('STOP'); b.lines(0.6)
if best:
    print('推荐 PID_S %g %g : 超调%.0f%% 稳态误差%+.1f%% 波动%.2f%% 振荡%.1fHz'
          % (best[1], best[2], best[3]['ov'], best[3]['err'], best[3]['rip'], best[3]['f']))
b.s.close()
