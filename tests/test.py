import pyplc
import time

def cb(pkt):
    print(pkt)

plc = pyplc.PyPlc()
plc.cs=8
plc.ldo=20
plc.rst=1
plc.irq=22
plc.setrxcb(cb)
plc.speed=2000000
plc.open(0,0)
a = bytearray("Test 123", "UTF-8")
time.sleep(1)
plc.tx(a)
time.sleep(2)
print(plc.ldo)
print(plc.rst)
plc.close()