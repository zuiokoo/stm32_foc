"""速度环参数扫描：CAPS 抓 2s（200Hz），扫 (kp, ki)，逐点给超调/上升/稳态误差/振荡。
每个点前先 SPD_OFF + IQ 0 让转子惰行，再从 ~0 阶跃到给定，保证初始条件一致。
自带发散保护。
"""
import serial, time, math
import numpy as np

PORT = 'COM7'
REF = 30.0        # rad/s 机械 ≈ 286 rpm
PTS = [(0.0,0.1), (0.00025,0.1), (0.0005,0.1), (0.001,0.1), (0.002,0.1), (0.0005,0.3), (0.0005,0.5), (0.001,0.5)]


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
            for k in ('SPD', 'SPD_REF', 'IQ', 'EXTRACLIP', 'SPD_EN'):
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
                            print('   !! 发散 spd=%.1f iq=%.2f -> 已 SPD_OFF' % (data[-1][1], data[-1][3]))
                            return None
        return data


def judge(d, ref):
    a = np.array(d)
    sp, iq = a[:, 1], a[:, 3]
    ss = sp[-100:]                      # 最后 0.5s
    err = 100 * (ss.mean() - ref) / ref
    ripple = 100 * (np.ptp(ss) / 2) / ref          # 半峰峰/给定
    pk = sp[:250].max()
    ov = 100 * (pk - ref) / ref
    i10 = next((i for i, v in enumerate(sp) if v >= 0.1 * ref), None)
    i90 = next((i for i, v in enumerate(sp) if v >= 0.9 * ref), None)
    rise = (i90 - i10) * 5 if (i10 is not None and i90 is not None) else -1
    # 建立时间：之后所有点都在 ±5% 内
    st = -1
    if i90 is not None:
        for i in range(i90, len(sp)):
            if np.all(np.abs(sp[i:] - ref) <= 0.05 * ref):
                st = i * 5
                break
    return dict(err=err, ripple=ripple, ov=ov, rise=rise, st=st,
                iq_ss=iq[-100:].mean(), iq_pk=iq.max())


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.lines(0.6)
b.send('STOP'); b.lines(0.3)
b.send('SPD_OFF'); b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)
# ---- 确认真的 RUNNING：CLEAR+RUN 后给小 iq，看转速有没有起来；丢命令就重试 ----
ok = False
for attempt in range(4):
    b.send('CLEAR'); b.lines(0.4)
    b.send('RUN');   b.lines(0.6)
    b.send('IQ 0.3'); b.lines(1.5)
    st = b.status()
    spd = st.get('SPD', 0)
    print('  尝试%d: SPD=%.1f rad/s IQ_meas=%.3f' % (attempt + 1, spd, st.get('IQ', 0)))
    if abs(spd) > 20:
        ok = True
        break
    b.lines(0.5)
if not ok:
    print('!! RUN 没起来或转子被夹（连续 4 次小 iq 都没转速），退出')
    b.send('IQ 0.0'); b.send('STOP'); b.s.close(); raise SystemExit
print('转子自由、环路已 RUNNING ✓')
b.send('IQ 0.0'); b.lines(3.0)      # 惰行

print('\n%-16s %7s %8s %8s %8s %8s %8s %8s' % ('(kp, ki)', '超调%', '上升ms', '建立ms', '稳态误差%', '波动±%', 'iq稳态', 'iq峰'))
best = None
for kp, ki in PTS:
    b.send('SPD_OFF'); b.lines(0.2)
    b.send('IQ 0.0'); b.lines(0.2)
    b.send('PID_S %g %g' % (kp, ki)); b.lines(0.2)
    b.lines(1.2)                                   # 惰行到低速，初始条件一致
    d = b.caps(REF)
    if d is None:
        print('%-16s  失败/发散' % ('(%.4g,%.3g)' % (kp, ki)))
        break
    j = judge(d, REF)
    print('%-16s %+7.1f %8d %8d %+8.2f %8.2f %8.3f %8.3f'
          % ('(%.4g,%.3g)' % (kp, ki), j['ov'], j['rise'], j['st'], j['err'], j['ripple'], j['iq_ss'], j['iq_pk']))
    score = abs(j['err']) + j['ripple'] + max(0, j['ov'] - 5) * 0.3 + max(0, j['st']) / 100.0
    if best is None or score < best[0]:
        best = (score, kp, ki, j)
    b.lines(0.3)

print('\n--- 收尾 ---')
b.send('SPD_OFF'); b.lines(0.3)
b.send('IQ 0.0'); b.lines(0.2)
b.send('STOP'); b.lines(0.8)
if best:
    print('推荐: PID_S %g %g   (超调 %.0f%%, 上升 %d ms, 稳态误差 %+.1f%%, 波动 ±%.1f%%)'
          % (best[1], best[2], best[3]['ov'], best[3]['rise'], best[3]['err'], best[3]['ripple']))
b.s.close()
