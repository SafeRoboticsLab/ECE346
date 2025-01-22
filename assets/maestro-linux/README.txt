Pololu Maestro USB Servo Controller Linux Software

Release Date: 2020-10-02
http://www.pololu.com/


== Summary ==

This binary release contains the Maestro Control Center
(MaestroControlCenter) and the Maestro command-line utility
(UscCmd).  These programs allow you to configure and control the
Maestro over USB.


== Prerequisites ==

You will need to download and install these packages:

  libusb-1.0-0-dev mono-runtime libmono-system-windows-forms4.0-cil

In Ubuntu, you can do this with the command:

  sudo apt-get install libusb-1.0-0-dev mono-runtime libmono-system-windows-forms4.0-cil


== USB Configuration ==

You will need to copy the file 99-pololu.rules to /etc/udev/rules.d/
in order to grant permission for all users to use Pololu USB devices.
Then, run

  sudo udevadm control --reload-rules

to make sure the rules get reloaded.  If you already plugged in
a Pololu USB device, you should unplug it at this point so the new
permissions will get applied later when you plug it back in.


== Running the programs ==

You can run the programs by typing one of the following commands:

   ./MaestroControlCenter
   ./UscCmd

If you get an error message that says "cannot execute binary file",
then try running the program with the mono command, for example:

   mono ./UscCmd


== Source Code ==

The C# source code for UscCmd is available in the Pololu USB Software
Development Kit, available at:

  http://www.pololu.com/docs/0J41
