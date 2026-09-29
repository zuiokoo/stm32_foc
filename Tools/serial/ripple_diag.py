"""电流环纹波溯源：判断 iq 的 ±5% 波动是
   (1) 转速引起的角度滞后（编码器 5ms 才更新一次）
   (2) 真实 PWM 纹波 / 采样点贴窗口边缘
   (3) 随机噪声（ADC/运放）
思路：先做 ID 测试（纯 d 轴无转矩 -> 转子不动、无 EMF、调制浅），
      这是最干净的"纯纹波+噪声"底噪；再做 IQ 测试对比（会引入转动）。
用固件自带的 RAM 抓取（CAP，400 点 x 50us = 20ms，覆盖 18ms 稳态）。
只发命令、只读回传，不发任何固件里没有的命令。
"""
import serial, time, sys, math

PORT = 'COM7'
rows_id = []
rows_iq = []


class Board:
    def __init__(self, port=PORT):
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.buf = []

    def send(self, cmd):
        self.s.write((cmd + '\r\n').encode())
        time.sleep(0.05)

    def lines(self, dur):
        out = []
        t0 = time.time()
        while time.time() - t0 < dur:
            ln = self.s.readline()
            if ln:
                out.append(ln.decode('ascii', 'replace').rstrip())
        return out

    def waitfor(self, sub, timeout=8.0, collect=None):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ln = self.s.readline()
            if ln:
                txt = ln.decode('ascii', 'replace').rstrip()
                if collect is not None:
                    collect.append(txt)
                if sub in txt:
                    return True
        return False

    def capture(self, cmd, timeout=8.0):
        """CAP -> 发 cmd 触发记录 -> 收 400 行 C ..."""
        self.send('CAP')
        got_armed = self.waitfor('CAP ARMED', 2.0)
        self.send(cmd)
        ok = self.waitfor('CAP BEGIN', timeout)
        if not ok:
            return None, got_armed
        data = []
        t0 = time.time()
        while time.time() - t0 < 6.0 and len(data) < 400:
            ln = self.s.readline()
            if not ln:
                continue
            txt = ln.decode('ascii', 'replace').strip()
            if txt.startswith('CAP END'):
                break
            if txt.startswith('C '):
                p = txt.split()
                if len(p) == 8:
                    data.append([float(x) for x in p[2:8]])
        return data, got_armed


def stats(seq):
    n = len(seq)
    m = sum(seq) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in seq) / n)
    lo, hi = min(seq), max(seq)
    # 相邻样本差（白噪声判据：diff std ≈ sqrt(2)*sd；低频波动：diff std << sqrt(2)*sd）
    d = [seq[i + 1] - seq[i] for i in range(n - 1)]
    dm = sum(d) / len(d)
    dsd = math.sqrt(sum((x - dm) ** 2 for x in d) / len(d))
    # 符号翻转次数（每 50us 一次采样）
    flips = sum(1 for i in range(1, n) if (seq[i] - m) * (seq[i - 1] - m) < 0)
    return dict(n=n, mean=m, std=sd, pp=hi - lo, lo=lo, hi=hi,
                dstd=dsd, flips=flips)


def show(tag, data, drop=200):
    if not data:
        print('  %s: 无数据' % tag)
        return
    s = data[drop:]                      # 丢掉前 10ms（阶跃过程）
    idr = [r[0] for r in s]
    iqr = [r[1] for r in s]
    d = [r[2] for r in s]
    q = [r[3] for r in s]
    vd = [r[4] for r in s]
    vq = [r[5] for r in s]
    siq, sid = stats(q), stats(d)
    svq, svd = stats(vq), stats(vd)
    vmag = [math.hypot(a, b) for a, b in zip(vd, vq)]
    print('  %s  (稳态 %d 点 = %.1f ms, 给定 id=%+.3f iq=%+.3f)' %
          (tag, len(s), len(s) * 0.05, idr[-1], iqr[-1]))
    print('    iq: 均值%+.4f  p-p %.4f  std %.4f  | 相邻差std %.4f (白噪声理论 %.4f)  翻转%d次'
          % (siq['mean'], siq['pp'], siq['std'], siq['dstd'], math.sqrt(2) * siq['std'], siq['flips']))
    print('    id: 均值%+.4f  p-p %.4f  std %.4f  | 相邻差std %.4f (白噪声理论 %.4f)  翻转%d次'
          % (sid['mean'], sid['pp'], sid['std'], sid['dstd'], math.sqrt(2) * sid['std'], sid['flips']))
    print('    vq: 均值%+.4f  p-p %.4f    vd: 均值%+.4f  p-p %.4f   |V| 均值 %.4f V'
          % (svq['mean'], svq['pp'], svd['mean'], svd['pp'], sum(vmag) / len(vmag)))
    # 前 12 点原样打印，肉眼看形状
    print('    iq 前12点: ' + ' '.join('%+.4f' % x for x in q[:12]))
    print('    id 前12点: ' + ' '.join('%+.4f' % x for x in d[:12]))
    return dict(iq=siq, id=sid, vq=svq, vd=svd)


def read_angle_msgs(b, n=10, gap=0.12):
    """连续 STATUS 读电角度，用来看转子转速（电角度回绕在 0~2pi）"""
    ang = []
    for _ in range(n):
        b.send('STATUS')
        got = None
        t0 = time.time()
        while time.time() - t0 < 1.0:
            ln = b.s.readline()
            if not ln:
                continue
            t = ln.decode('ascii', 'replace').strip()
            if t.startswith('ANGLE'):
                got = float(t.split()[1])
            if t == 'IQ_REF' or t.startswith('IQ_REF') or t.startswith('ID_REF'):
                pass
            if t.startswith('STATE'):
                break
        if got is not None:
            ang.append(got)
        time.sleep(gap)
    return ang


def unwrap(seq):
    out = [seq[0]]
    for a in seq[1:]:
        d = a - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def main():
    b = Board()
    print('OPEN_OK %s' % PORT)
    print('--- 被动读 2s（看当前状态）---')
    for l in b.lines(2.0):
        print('   ', l)

    b.send('VOFA OFF')
    b.send('CLEAR')
    b.lines(0.5)

    print('--- ALIGN ---')
    a = []
    if not b.waitfor('ALIGN: done', 8.0, a):
        print('ALIGN 超时/失败:'); print('\n'.join(a)); return
    print('   ', [x for x in a if 'ALIGN' in x])

    b.send('RUN')
    print('--- RUN ---')
    print('   ', b.lines(0.8))

    # 清零点：给定 0 时电流应该在 0 附近（顺便看零点漂移）
    b.send('ID 0.0'); b.send('IQ 0.0')
    b.lines(0.5)

    print('--- 测试 A：ID 0.1（转子不动，最干净的底噪）---')
    d, _ = b.capture('ID 0.1')
    show('ID 0.1', d)
    b.lines(0.3)

    print('--- 测试 B：ID 0.5（同条件，看纹波随电流的变化）---')
    d, _ = b.capture('ID 0.5')
    r_id5 = show('ID 0.5', d)
    b.lines(0.3)

    # ID 测试期间转子有没有动
    print('   ID 0.5 期间角度采样:', ['%.3f' % x for x in read_angle_msgs(b, 6)])

    print('--- 回到 0，测转子是否被固定（小 iq 看角度是否漂）---')
    b.send('ID 0.0'); b.lines(0.3)
    b.send('IQ 0.05'); b.lines(0.6)
    ang = read_angle_msgs(b, 8)
    if len(ang) >= 2:
        u = unwrap(ang)
        drift = u[-1] - u[0]
        print('   IQ 0.05: 电角度漂移 %+.3f rad over %.2fs -> %.1f rad/s elec = %.0f rpm mech'
              % (drift, 0.12 * (len(ang) - 1), drift / (0.12 * (len(ang) - 1)),
                 drift / (0.12 * (len(ang) - 1)) / 7.0 * 60 / (2 * math.pi)))

    print('--- 测试 C：IQ 0.5（复现你的工况）---')
    d, _ = b.capture('IQ 0.5')
    show('IQ 0.5', d)

    print('--- 停机 ---')
    b.send('IQ 0.0'); b.send('STOP')
    print('   ', b.lines(1.0))
    b.s.close()


main()
