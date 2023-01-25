#!/usr/bin/env python

import sys
import os
import time
import serial
import maestro

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# chose an implementation, depending on os
#~ if sys.platform == 'cli':
#~ else:
if os.name == 'nt':  # sys.platform == 'win32':
    from serial.tools.list_ports_windows import comports
elif os.name == 'posix':
    from serial.tools.list_ports_posix import comports
#~ elif os.name == 'java':
else:
    raise ImportError("Sorry: no implementation for your platform ('{}') available".format(os.name))
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def pololu_devices():
    for info in comports(include_links=False):
        port, desc, hwid = info
        if 'Pololu' in desc:
            yield info
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

devices = maestro.getConnectedDevices()

if len(devices) == 0:
    sys.exit('No Maestro devices connected')

print("Pololu Maestro firmware upgrade")
for i,device in enumerate(devices):
    print(f"{i}: {device.getName()}")
num = input("Enter device number to upgrade: ")

if not num.isdigit() or not 0 <= int(num) < len(devices):
    sys.exit('wrong input')

print("Restarting bootloader...")
devices[int(num)].startBootloader()
time.sleep(3)

devices = pololu_devices()
if len(devices) == 0:
    sys.exit('No Maestro devices in bootloader mode detected')

for i,(port, desc, hwid) in enumerate(devices):
    print(f"{i}: {port}")
    print(f"    desc: {desc}")
    print(f"    hwid: {hwid}")

num = input("Enter device number to upgrade: ")
if not num.isdigit() or not 0 <= int(num) < len(devices):
    sys.exit('wrong input')

ser = serial.Serial(devices[int(num)].port)
print(ser.name)         # check which port was really used
ser.write(b'fwbootload')     # write a string
line = ser.readline()
print(line)
ser.close() 
