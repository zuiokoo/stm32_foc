"""只读探测：打开 COM7，抓 1.5s 日志，再发一条 STATUS 读状态。不发任何驱动命令。"""
import sys, time
try:
    import serial
except ImportError:
    print("NO_PYSERIAL"); sys.exit(1)

PORT = 'COM7'
try:
    s = serial.Serial(PORT, 115200, timeout=0.3)
except Exception as e:
    print("OPEN_FAIL:", e)
    sys.exit(2)

print("OPEN_OK", PORT)
time.sleep(1.5)
d = s.read(4000)
print("--- drain(%d bytes) ---" % len(d))
print(d.decode('ascii', 'replace'))

# 只读：STATUS（打印 ANGLE/ID/IQ/ID_REF/IQ_REF，不改变任何状态）
s.write(b'STATUS\n')
time.sleep(0.6)
print("--- STATUS reply ---")
print(s.read(4000).decode('ascii', 'replace'))
s.close()
print("CLOSED_OK")
