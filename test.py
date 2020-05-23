import pyplc
import time

plc = pyplc.PyPlc()


print("---1")
plc.cs=8
plc.ldo=20
plc.rst=1
plc.irq=22
plc.speed=2000000
plc.open(0,0)

print("---2")
a = bytearray("Test 123", "UTF-8")
time.sleep(2)
plc.tx(a)
time.sleep(2)
print("---3")
print(plc.ldo)
print(plc.rst)
plc.close()