PyPlc
=====

This project contains a python module for interfacing with ATPL360 device from user space via the spidev linux kernel driver.

All code is MIT licensed unless explicitly stated otherwise.

Usage
-----

```python
import pyplc
plc = pyplc.PyPlc()
# Set additional pins
plc.ldo = 28
plc.rst = 29
plc.open(bus, device)
to_send = [0x01, 0x02, 0x03]
plc.xfer(to_send)
```

Settings
--------

* `ldo` - set LDO control pin number
* `rst` - set Reset pin number
* `speed` - set SPI bus speed

Methods
-------

    open(bus, device)

Connects to the specified SPI device, opening `/dev/spidev<bus>.<device>` and `/sys/class/gpio/*`, checks PLC state and boots if neseccary

    tx(bytearray)

Transmits data packet over PLC bus

    rx - ?? ToDo callback
