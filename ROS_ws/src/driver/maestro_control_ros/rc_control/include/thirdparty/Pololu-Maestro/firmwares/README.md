# Upgrading Firmware

The firmware is the program that runs on the Maestro. The Maestro has field-upgradeable firmware that can be easily updated with bug fixes or new features. There are four types of Maestros: the Micro Maestro 6-Channel USB Servo Controller (usc02a), the Mini Maestro 12-Channel USB Servo Controller (usc03a), the Mini Maestro 18-Channel USB Servo Controller (usc03b), and the Mini Maestro 24-Channel USB Servo Controller (usc03c). Each type of Maestro has its own firmware and you cannot load the firmware for one type onto another.

# Upgrade Instructions

You can determine the version of your Maestro’s firmware by running the Maestro Control Center, connecting to a Maestro, and looking at the firmware version number which is displayed in the upper left corner next to the “Connected to” drop-down box. If you do not already have the latest version (1.03 for Mini Maestros, 1.04 for the Micro Maestro), you can upgrade by following the instructions below:

All of your settings will be reset to default values during the firmware upgrade.

1. Determine which type of Maestro you have either by counting the number of channels or by looking at the name that appears in your Device Manager. There are four types of Maestro: the Micro Maestro 6-Channel USB Servo Controller (usc02a), the Mini Maestro 12-Channel USB Servo Controller (usc03a), the Mini Maestro 18-Channel USB Servo Controller (usc03b), and the Mini Maestro 24-Channel USB Servo Controller (usc03c).
2. Start your Maestro in bootloader mode (see Section 4.f.1 from the maestro.pdf).
3. Run the FirmwareUpgradeUtility.py python script

If you have problems during or after the firmware upgrade, then it is possible that you loaded the wrong firmware onto your Maestro or some other problem corrupted the firmware. The solution is to retry the firmware upgrade procedure above. 