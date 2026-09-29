"""速度环测试：CAPS 抓 2s 波形（200Hz），看阶跃响应并给出调参建议。
自带失控保护：一旦发现转速发散（|spd| > 5×给定 且 iq 顶到限幅），立刻 SPD_OFF。
用法：python speed_test.py [参考转速 rad/s]   默认测 2 和 5 rad/s
"""
import serial, time, sys
import numpy as np

PORT = 'COM7'


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
            if ln:
                t = ln.decode('ascii', 'replace').rstrip()
                if sub in t:
                    return True
        return False

    def status(self):
        """返回 dict: SPD/SPD_REF/IQ/IQ_REF/STATE"""
        self.send('STATUS')
        d, t0 = {}, time.time()
        while time.time() - t0 < 1.0:
            ln = self.s.readline()
            if not ln:
                continue
            t = ln.decode('ascii', 'replace').strip()
            for k in ('SPD', 'SPD_REF', 'IQ', 'IQ_REF', 'ANGLE', 'SPD_EN'):
                if t.startswith(k + ' '):
                    try:
                        d[k] = float(t.split()[1])
                    except ValueError:
                        pass
        return d

    def caps(self, speed_ref):
        """CAPS 之后发 SPD，抓 2s"""
        self.send('CAPS')
        if not self.waitfor('CAPS ARMED', 2.0):
            print('   CAPS 未 armed'); return None
        self.send('SPD %.4f' % speed_ref)
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
                    # 失控保护：边收边看
                    if len(data) % 50 == 0:
                        sp, iq = data[-1][1], data[-1][3]
                        if abs(sp) > 5 * abs(speed_ref) + 20 and abs(iq) > 0.55:
                            self.send('SPD_OFF')
                            self.send('STOP')
                            print('   !! 发散：spd=%.1f iq=%.2f -> 已 SPD_OFF' % (sp, iq))
                            return None
        return data


def analyse(tag, d, ref):
    if not d or len(d) < 100:
        print('  %s: 数据不足' % tag); return
    a = np.array(d)
    spr, sp, iqr, iq = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    print('  %s   给定 %.2f rad/s (%.0f rpm)' % (tag, ref, ref * 60 / (2 * np.pi)))
    print('    稳态: spd %.3f (误差 %+.2f%%)  iq 均值 %.3f  波动 ±%.3f' %
          (sp[-100:].mean(), 100 * (sp[-100:].mean() - ref) / ref, iq[-100:].mean(), sp[-100:].std()))
    print('    峰值 spd %.3f (超调 %+.0f%%)   iq 峰值 %.3f/%+.3f' %
          (sp.max() if ref > 0 else sp.min(),
           100 * (sp.max() - ref) / ref if ref > 0 else 100 * (sp.min() - ref) / ref,
           iq.max(), iq.min()))
    # 上升时间：从给定量 10% 到 90%
    tgt = [i for i, v in enumerate(sp) if v >= 0.9 * ref]
    t10 = [i for i, v in enumerate(sp) if v >= 0.1 * ref]
    if tgt and t10:
        print('    上升(10%%->90%%) %.0f ms    2%% 建立时间 %s ms' %
              ((tgt[0] - t10[0]) * 5,
               next((i * 5 for i, v in enumerate(sp[tgt[0]:]) if abs(v - ref) < 0.02 * abs(ref)), -1)))
    print('    前12点 spd: ' + ' '.join('%.2f' % v for v in sp[:12]))
    print('    前12点 iq : ' + ' '.join('%.3f' % v for v in iq[:12]))


b = Board()
print('OPEN_OK')
b.send('VOFA OFF'); b.send('STOP'); b.lines(1.0)
b.send('CLEAR'); b.lines(0.3)
b.send('RUN')
print('RUN:', b.waitfor('RUN: pwm started', 3.0))
b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.5)
print('当前状态:', b.status())

refs = [float(x) for x in sys.argv[1:]] or [2.0, 5.0]
for ref in refs:
    print('\n--- 速度阶跃 -> %.2f rad/s ---' % ref)
    d = b.caps(ref)
    analyse('CAPS', d, ref)
    st = b.status()
    print('    实测 STATUS:', {k: round(v, 3) for k, v in st.items()})
    if d is None:
        break
    b.lines(0.3)

print('\n--- 收尾 ---')
b.send('SPD_OFF'); b.lines(0.3)
b.send('STOP'); b.lines(0.6)
b.s.close()
