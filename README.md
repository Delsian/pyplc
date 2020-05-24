PyPlc
=====

This project contains a python module for interfacing with ATPL360 device from user space via the spidev linux kernel driver.

All code is MIT licensed unless explicitly stated otherwise.

Usage
-----

```python
import pyplc
def cb(pkt):
    print(pkt)

plc = pyplc.PyPlc()
# Set additional pins
#plc.cs = 8
#plc.ldo = 20
#plc.rst = 1
plc.irq = 22
plc.setrxcb(cb)
plc.speed=2000000
plc.open(0,0,8,20,1)
a = bytearray("Test", "UTF-8")
plc.tx(a)
print(plc.ldo)
print(plc.rst)
print(plc.cs)
print(plc.irq)
plc.close()
```

Settings
--------

* `cs` - SPI CS software-controlled pin
* `ldo` - set LDO control pin number
* `rst` - set Reset pin number
* `irq` - set IRQ pin number
* `speed` - set SPI bus speed

Methods
-------

    open(bus, device[,cs=0, ldo=0, rst=0, irq=0])

Connects to the specified SPI device, opening `/dev/spidev<bus>.<device>` and `/sys/class/gpio/*`, checks PLC state and boots if neseccary.

    tx(bytearray)

Transmits data packet over PLC bus

    setrxcb(cb)
    
Set callback to function accepts bytearray as argument

    close()

Close SPI connection and unexport all pins
