"""解析 VOFA JustFloat 帧文件（每行 7 个 float + 00 00 80 7F 尾巴，行首可带 [hh:mm:ss.mmm]）。
布局: [iq_ref, iq, vd, vq, ia, ib, ic]
用法: python decode_frames.py <文件...>   不给文件则读 frames_1704.txt
再算 LSB 格点检查：原始（未滤波）读数应该精确落在 3.2235mA 整数倍上，
滤波后会出现亚 LSB 值——这是判断固件里滤波是否生效的直接证据。
"""
import struct, re, sys, os

LSB_A = (3.3 / 4095.0) / (0.005 * 50.0)     # 0.0032235 A


def load(path):
    frames = []
    for line in open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line:
            continue
        m = re.match(r'\[(\d\d:\d\d:\d\d\.\d+)\]\s+(.*)', line)
        ts = m.group(1) if m else ''
        body = m.group(2) if m else line
        try:
            b = bytes(int(x, 16) for x in body.split())
        except ValueError:
            continue
        i = b.rfind(b'\x00\x00\x80\x7f')
        if i < 28:
            continue
        frames.append((ts, struct.unpack('<7f', b[i - 28:i])))
    return frames


def report(path):
    fr = load(path)
    if not fr:
        print('%s: 没解析出帧' % os.path.basename(path)); return
    print('=== %s : %d 帧 (≈%.0f ms) ===' % (os.path.basename(path), len(fr), len(fr) * 5))
    iq = [v[1] for _, v in fr]
    vq = [v[3] for _, v in fr]
    vd = [v[2] for _, v in fr]
    ia = [v[4] for _, v in fr]
    ib = [v[5] for _, v in fr]
    ic = [v[6] for _, v in fr]

    def st(x):
        m = sum(x) / len(x)
        sd = (sum((a - m) ** 2 for a in x) / len(x)) ** 0.5
        return m, sd, max(x) - min(x)

    miq, siq, piq = st(iq)
    mvq, svq, pvq = st(vq)
    print('iq: 均值%+.4f  峰峰 %.4f (%.1f%%)  std %.4f (%.2f%%)' %
          (miq, piq, 100 * piq / miq, siq, 100 * siq / miq))
    print('vq: 均值%+.4f  峰峰 %.4f (%.1f%%)  std %.4f' % (mvq, pvq, 100 * pvq / mvq, svq))
    print('vd: 均值%+.4f  峰峰 %.4f' % (sum(vd) / len(vd), max(vd) - min(vd)))

    # LSB 格点：取 |i| 最大的那一相来查（它承载电流矢量）
    def lsb_err(x):
        return max(abs(a / LSB_A - round(a / LSB_A)) for a in x if abs(a) > 0.05)

    print('LSB 格点偏差(最大相): ia %.3f  ib %.3f  ic %.3f  LSB=%.4f mA  '
          '(≈0 = 原始未滤波；明显非 0 = 滤波已生效)'
          % (lsb_err(ia), lsb_err(ib), lsb_err(ic), LSB_A * 1000))
    print('iq 序列: ' + ' '.join('%.4f' % x for x in iq))
    print('vq 序列: ' + ' '.join('%.4f' % x for x in vq))
    d = [iq[i + 1] - iq[i] for i in range(len(iq) - 1)]
    print('iq 逐帧差 std %.4f' % (sum(x * x for x in d) / len(d)) ** 0.5)
    print()


for p in (sys.argv[1:] or ['frames_1704.txt']):
    if os.path.exists(p):
        report(p)
    else:
        report(os.path.join(os.path.dirname(os.path.abspath(__file__)), p))
