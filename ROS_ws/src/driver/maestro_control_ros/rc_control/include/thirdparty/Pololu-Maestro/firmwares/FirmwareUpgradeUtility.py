#!/usr/bin/env python

import sys
import os
import time

try:
    import serial
except ImportError:
    print(f'Pyserial is not installed for {sys.executable}.')
    raise

try:
    import serial.tools.list_ports as list_ports
except ImportError:
    print(f'The installed version ({sys.VERSION}) of pyserial appears to be too old (Python interpreter {sys.executable}).')
    raise

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def find_maestro_devices():
    devices = []
    for info in list_ports.comports(include_links=False):
        port, desc, hwid = info
        if 'Pololu' and 'Bootloader' in desc:
            devices.append(port)
    return sorted(devices)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + ' ' * (bar_len - filled_len)

    sys.stdout.write('[%s] %s%s %s\r' % (bar, percents, '%', status))
    sys.stdout.flush()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def flash(firmware, serialPort):
    # handshake
    text=b'fwbootload'
    serialPort.write(text)
    answer = serialPort.read(len(text))
    if answer != text.upper():
        sys.exit(f'Expected {text.upper()} got {answer}')

    print('Connected to Maestro bootloader.')

    print('Erasing existing Maestro firmware...')
    serialPort.write(b's')
    answer = serialPort.read()

    if answer != b'S':
        print(f"There was error erasing the old firmware.  Expected response 'S' from device but received response {answer}")
        raise

    print(f"Uploading new firmware...")
    Position=0
    while Position < len(firmware):
        serialPort.flush()
        serialPort.write(firmware[Position:Position+1000])
        Position+=min(1000, len(firmware) - Position)
        progress(Position, len(firmware))

    time.sleep(0.2)
    answer=serialPort.read(serialPort.in_waiting)

    if (len(answer) == 0 or answer[-1] != 124) and serialPort.read() != 124:
        print(f"Expected to receive the '|' character, but did not.")

    print(f"\nUpload completed successfully.")
    serialPort.write(b'*')
    time.sleep(0.2)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def main(argv):
    print(f'Pololu Maestro firmware upgrade utility\n')
    if len(argv) != 2:
        sys.exit(f'Usage: {argv[0]} firmware.pgm\n')

    with open(argv[1], mode='rb') as file:
        firmware = file.read()

    devices = find_maestro_devices()

    if len(devices) == 0:
        sys.exit(f'No Maestro devices in bootloader mode connected.')
    if len(devices) > 1:
        sys.exit(f'More than one Maestro devices in bootloader mode connected.')

    with serial.Serial(devices[0]) as serialPort:
        flash(firmware, serialPort)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if __name__ == '__main__':
    main(sys.argv)
