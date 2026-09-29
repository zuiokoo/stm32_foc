"""CAP 抓取 + 频域分析：用固件 RAM 抓取(400点 x 50us = 20ms)看电流环的真实波形。
回答两个问题：
 1) 4% 的波动是随机噪声，还是某个频率的振荡？
 2) vq/iq 的相位关系（200Hz 显示只能混叠，50us 才能看到真身）
流程：VOFA OFF -> CLEAR -> ALIGN -> RUN -> 三组 CAP（零电流 / ID0.5 / IQ0.5）-> STOP
"""
import serial, time, math, sys
import numpy as np

PORT = 'COM7'
FS = 20000.0          # 50us
DROP = 100            # 丢掉前 5ms（阶跃过程）


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

    def waitfor(self, sub, timeout=10.0):
        t0 = time.time()
        seen = []
        while time.time() - t0 < timeout:
            ln = self.s.readline()
            if ln:
                txt = ln.decode('ascii', 'replace').rstrip()
                seen.append(txt)
                if sub in txt:
                    return True, seen
        return False, seen

    def capture(self, cmd, timeout=8.0):
        self.send('CAP')
        ok, _ = self.waitfor('CAP ARMED', 2.0)
        if not ok:
            return None
        self.send(cmd)
        ok, seen = self.waitfor('CAP BEGIN', timeout)
        if not ok:
            print('   CAP 没触发:', seen[-3:])
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


def analyse(tag, data, r_known=3.53):
    if not data or len(data) < 200:
        print('  %s: 数据不足' % tag)
        return
    a = np.array(data[DROP:])          # [id_ref, iq_ref, id, iq, vd, vq]
    idr, iqr, idc, iqc, vd, vq = [a[:, i] for i in range(6)]
    print('  %s  (%d 点 = %.1f ms)' % (tag, len(a), len(a) * 0.05))
    for nm, x in (('iq', iqc), ('id', idc), ('vq', vq), ('vd', vd)):
        print('    %-3s 均值%+8.4f  std %7.4f  峰峰 %7.4f  (%.2f%%)' %
              (nm, np.mean(x), np.std(x), np.ptp(x), 100 * np.ptp(x) / (abs(np.mean(x)) + 1e-9)))

    # 频谱（去掉均值，Hann 窗）
    for nm, x in (('iq', iqc), ('vq', vq)):
        y = x - np.mean(x)
        w = np.hanning(len(y))
        sp = np.abs(np.fft.rfft(y * w)) * 2.0 / (w.sum())
        fr = np.fft.rfftfreq(len(y), 1.0 / FS)
        idx = np.argsort(sp[1:])[::-1][:6] + 1          # 忽略 DC
        top = ', '.join('%.0fHz:%.4f' % (fr[i], sp[i]) for i in idx[:6])
        band = []
        for lo, hi in ((0, 100), (100, 300), (300, 1000), (1000, 5000), (5000, 10000)):
            m = (fr >= lo) & (fr < hi)
            band.append('%.0f-%.0f' % (lo, hi))
            band[-1] += 'Hz:%.4f' % sp[m].max() if m.any() else 'Hz:-'
        print('    %s 频谱 top: %s' % (nm, top))
        print('    %s 分带峰值: %s' % (nm, '  '.join(band)))

    # vq 与 iq 的互相关（找真实滞后）
    x = vq - vq.mean()
    y = iqc - iqc.mean()
    c = np.correlate(x, y, 'full') / (len(x) * np.std(x) * y.std() + 1e-12)
    lags = np.arange(-len(x) + 1, len(x))
    m = (lags >= -40) & (lags <= 40)
    k = np.argmax(np.abs(c[m]))
    lag = lags[m][k]
    print('    vq×iq 互相关: lag0=%+.2f  峰值 lag=%+d 样本 (%+.0f us) 值%+.2f' %
          (c[len(x) - 1], lag, lag * 50, c[m][k]))

    # 白噪声判据：相邻样本差 std 与 sqrt(2)*std 比
    d = np.diff(iqc)
    print('    iq 相邻差 std %.4f  白噪声理论 %.4f  比值 %.2f  (<<1 = 低频波动/振荡, ≈1 = 白噪声)'
          % (np.std(d), np.sqrt(2) * iqc.std(), np.std(d) / (np.sqrt(2) * iqc.std() + 1e-12)))
    print('    真实均值校验: iq均值%+.4f  vq均值/R(%.2fΩ)=%+.4f' %
          (iqc.mean(), r_known, vq.mean() / r_known))
    print('    iq 前 20 点: ' + ' '.join('%+.4f' % v for v in iqc[:20]))
    print()


b = Board()
print('OPEN_OK')
b.send('VOFA OFF')
b.send('STOP')
b.lines(1.2)
print('--- 状态 ---')
for l in b.lines(0.5):
    print('   ', l)

b.send('CLEAR')
b.lines(0.4)
b.send('STATUS')
print('   ', b.lines(0.6))

# 注意：ALIGN 只在 CALIBRATING->ALIGN 那个状态下被执行，板上跑过一次之后
# 状态就固定在 READY/RUNNING，ALIGN 命令会被忽略。零点偏移已在 RAM 里且转子固定，
# 不需要重新对齐，直接 RUN。
print('--- RUN ---')
b.send('RUN')
ok, seen = b.waitfor('RUN: pwm started', 3.0)
print('   ', [x for x in seen if 'RUN' in x] or seen[-3:])
b.send('ID 0.0'); b.send('IQ 0.0'); b.lines(0.6)

print('--- 捕获 1/3：零电流（环路在跑，给定 0）---')
d = b.capture('ID 0.0')
analyse('零电流', d)
b.lines(0.3)

print('--- 捕获 2/3：ID 0.5（无转矩、转子不动、调制浅）---')
d = b.capture('ID 0.5')
analyse('ID 0.5', d)
b.lines(0.3)

print('--- 捕获 3/3：IQ 0.5（你的工况）---')
b.send('ID 0.0'); b.lines(0.2)
d = b.capture('IQ 0.5')
analyse('IQ 0.5', d)

print('--- 停机 ---')
b.send('IQ 0.0'); b.send('STOP')
print('   ', b.lines(0.8))
b.s.close()
