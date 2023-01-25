
Pololu Maestro USB Servo Controller library for Windows, Linux, and macOS
=========================================================================

![C/C++ CI](https://github.com/papabricole/Pololu-Maestro/workflows/C/C++%20CI/badge.svg)
[![GitHub Releases](https://img.shields.io/github/release/papabricole/Pololu-Maestro.svg)](https://github.com/papabricole/Pololu-Maestro/releases)

## Description

This package contains the code you need for making your own
applications that control Pololu Maestro USB Devices.  All of the code
uses the devices' native USB interfaces.

The supported devices are:

  - Micro Maestro 6-Channel USB Servo Controller (#1350, #1351)
  - Mini Maestro 12-Channel USB Servo Controller (#1352, #1353)
  - Mini Maestro 18-Channel USB Servo Controller (#1354, #1355)
  - Mini Maestro 24-Channel USB Servo Controller (#1356, #1357)

## Upgrading Firmware

See the firmwares subdirectory for the instructions.

## Prerequisites

**On all platforms**

* A compiler with C++11 support
* CMake >= 3.11.0

**On Linux**

* libusb-dev
* python-dev

## How to Use

### C++

    #include <maestro/Device.h>
    
    std::vector<Maestro::Device> devices = Maestro::Device::getConnectedDevices();

    devices[0].setAcceleration(0, 4);  // set servo 0 acceleration to 4
    devices[0].setTarget(0, 6000);     // set servo to move to center position
    devices[0].setSpeed(1, 10);        // set servo 1 speed to 10

### Python

    import maestro
    devices = maestro.getConnectedDevices()

    if len(devices) == 0:
        print("No Maestro devices connected")
        exit()

    for device in devices:
        print(f"name: {device.getName()} #channels: {device.getNumChannels()}")
        for c in range(device.getNumChannels()):
            channel=device.getChannelSettings(c)
            print(f"#{c} mode: {channel.mode} min: {channel.minimum} max: {channel.maximum} speed: {channel.speed}")

    # select first Maestro device
    device=devices[0]
    device.setAcceleration(0, 4) # set servo 0 acceleration to 4
    device.setTarget(0, 6000)    # set servo to move to center position
    device.setSpeed(1, 10)       # set servo 1 speed to 10

    # compile and upload a script
    with open('blink.txt', 'r') as content_file:
        script = content_file.read()

    program = maestro.Program(script=script, isMiniMaestro=False)

    device.writeScript(program.getByteList())