import pyplc

plc = pyplc.PyPlc()


print("---1")
plc.ldo=12
plc.rst=17

print("---2")
a = bytearray("Test", "UTF-8")
#print(plc.tx("Test"))

print("---3")
print (plc.ldo)
print( plc.rst)