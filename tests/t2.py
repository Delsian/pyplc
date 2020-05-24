import pyplc
import time

def cb(pkt):
    print(pkt.hex())

plc = pyplc.PyPlc()
plc.setrxcb(cb)
plc.speed=2000000
plc.irq = 22
plc.open(0,0,8,20,1)
print(plc.ldo)
print(plc.rst)
print(plc.cs)
print(plc.irq)
time.sleep(2)
plc.close()