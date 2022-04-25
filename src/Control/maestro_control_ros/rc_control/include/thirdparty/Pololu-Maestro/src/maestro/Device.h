/*! \file Device.h
 *
 * This is the main header file for the Pololu Maestro Servo Controller library.
 *
 * For an overview of the library's features, see
 * https://github.com/papabricole/Pololu-Maestro. That is the main repository for
 * the library.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Maestro {
class Device {
   public:
    enum Parameter : uint8_t;

    enum class SerialMode {
        /// On the Command Port, user can send commands and receive responses.
        /// TTL port/UART are connected to make a USB-to-serial adapter.</summary>
        SERIAL_MODE_USB_DUAL_PORT = 0,

        /// On the Command Port, user can send commands to Maestro and
        /// simultaneously transmit bytes on the UART TX line, and user
        /// can receive bytes from the Maestro and the UART RX line.
        /// TTL port does not do anything.
        SERIAL_MODE_USB_CHAINED = 1,

        /// On the UART, user can send commands and receive reponses after
        /// sending a 0xAA byte to indicate the baud rate.
        /// Command Port receives bytes from the RX line.
        /// TTL Port does not do anything.
        SERIAL_MODE_UART_DETECT_BAUD_RATE = 2,

        /// On the UART, user can send commands and receive reponses
        /// at a predetermined, fixed baud rate.
        /// Command Port receives bytes from the RX line.
        /// TTL Port does not do anything.
        SERIAL_MODE_UART_FIXED_BAUD_RATE = 3
    };

    /// This struct represents all the settings for a Maestro.
    struct DeviceSettings {
        uint8_t firmwareVersionMinor;
        uint8_t firmwareVersionMajor;

        /// The number of servo ports available (0-5).  This, along with the
        /// servoPeriod, determine the "maximum maximum pulse width".
        uint8_t servosAvailable;

        /// This setting only applies to the Micro Maestro.
        /// For the Mini Maestro, see miniMaestroServoPeriod.
        ///
        /// The total time allotted to each servo channel, in units of
        /// 256/12 = 21.33333 us.  The unit for this one are unusual, because
        /// that is the way it is stored on the device and its unit is not
        /// a multiple of 4, so we would have inevitable rounding errors if we
        /// tried to represent it in quarter-microseconds.
        ///
        /// Default is 156, so with 6 servos available you get ~20ms between
        /// pulses on a given channel.
        uint8_t servoPeriod;

        /// This setting only applies to the Mini Maestro.
        /// For the Micro Maestro, see microMaestroServoPeriod.
        ///
        /// The length of the time period in which the Mini Maestro sends pulses
        /// to all the enabled servos, in units of quarter microseconds.
        ///
        /// Valid values for this parameter are 0 to 16,777,215.  But
        ///
        /// Default is 80000, so each servo receives a pulse every 20 ms (50 Hz).
        uint32_t miniMaestroServoPeriod;

        /// This setting only applied to the Mini Maestro.
        /// The non-multiplied servos have a period specified by
        /// miniMaestroServoPeriod. The multiplied servos have a period specified by
        /// miniMaestroServoPeriod*servoMultiplier.
        ///
        /// Valid values for this parameter are 1 to 256.
        uint16_t servoMultiplier;

        /// Determines how serial bytes flow between the two USB COM ports, the TTL
        /// port, and the Maestro's serial command processor.
        SerialMode serialMode;

        /// The fixed baud rate, in units of bits per second.  This gets stored in a
        /// different format on the usc.cs, so there will be rounding errors
        /// which get bigger at higher baud rates, but they will be less than
        /// 1% for baud rates of 120000 or less.
        ///
        /// This parameter only applies if serial mode is USB UART Fixed Baud.
        ///
        /// All values above 184 are valid, but values significantly higher than
        /// 250000 are subject to high rounding errors and the usc firmware might
        /// not be able to keep up with those higher data rates.  If the baud rate
        /// is too high and the firmware can't keep up, the Maestro will indicate
        /// this to you by generating a serial overrun or buffer full error.
        uint32_t fixedBaudRate;

        /// If true, then you must send a 7-bit CRC byte at the end of every serial
        /// command (except the Mini SSC II command).
        uint8_t enableCrc;

        /// If true, then the Maestro will never go to sleep.  This lets you power
        /// the processer off of USB even when the computer has gone to sleep and
        /// put all of its USB devices in the suspend state.
        uint8_t neverSuspend;

        /// The serial device number used to identify this device in Pololu protocol
        /// commands.  Valid values are 0-127, default is 12.
        uint8_t serialDeviceNumber;

        /// The offset used to determine which Mini SSC commands this device will
        /// respond to.  The second byte of the Mini SSC command contains the servo
        /// number; the correspondence between servo number and maestro number (0-5)
        /// is servo# = miniSSCoffset + channel#.  Valid values are 0-254.
        uint8_t miniSscOffset;

        /// The time it takes for a serial timeout error to occur, in units of 10
        /// ms. A value of 0 means no timeout error will occur.  All values 0-65535
        /// are valid.
        uint16_t serialTimeout;

        /// True if the script should not be started when the device starts up.
        /// False if the script should be started.
        uint8_t scriptDone;

        /// If true, this setting enables pullups for each channel 18-20 which
        /// is configured as an input.  This makes the input value be high by
        /// default, allowing the user to connect a button or switch without
        /// supplying their own pull-up resistor.  Thi setting only applies to
        /// the Mini Maestro 24-Channel Servo Controller.
        uint8_t enablePullups;
    };

    enum class ChannelMode : uint8_t { SERVO = 0, SERVO_MULTIPLIED = 1, OUTPUT = 2, INPUT = 3 };

    enum class HomeMode : uint8_t { OFF = 0, IGNORE = 1, GOTO = 2 };

    /// An object that represents the settings for one servo,
    /// e.g. the information in the Settings tab.  One of these objects
    /// corresponds to one ServoSettingsControl.
    struct ChannelSettings {
        /// Type (0=servo, 1=servo multiplied, 2=output, 3=input).
        ChannelMode mode;

        /// HomeType (0=off, 1=ignore, 2=goto).
        HomeMode homeMode;

        /// Home position: the place to go on startup.
        /// If type==servo, units are 0.25 us (qus).
        /// If type==output, the threshold between high and low is 1500.
        ///
        /// This value is only saved on the device if homeType == Goto.
        uint16_t home;

        /// Minimum (units of 0.25 us, but stored on the device in units of 16 us).
        uint16_t minimum;

        /// Maximum (units of 0.25 us, but stored on the device in units of 16 us).
        uint16_t maximum;

        /// Neutral: the center of the 8-bit set target command (value at 127).
        /// If type==servo, units are 0.25 us (qus).
        /// If type==output, the threshold between high and low is 1500.
        uint16_t neutral;

        /// Range: the +/- extent of the 8-bit command.
        ///   8-bit(254) = neutral + range,
        ///   8-bit(0) = neutral - range
        /// If type==servo units are 0.25 us (qus) (but stored on device in
        /// units of 127*0.25us = 31.75 us.
        /// Range = 0-127*255 = 0-32385 qus.
        /// Increment = 127 qus
        uint16_t range;

        /// Speed: the maximum change in position (qus) per update.  0 means no
        /// limit. Units depend on your settings. Stored on device in this format:
        /// [0-31]*2^[0-7] Range = 0-31*2^7 = 0-3968. Increment = 1.
        ///
        /// Note that the *current speed* is stored on the device in units
        /// of qus, and so it is not subject to the restrictions above!
        /// It can be any value 0-65535.
        uint16_t speed;

        /// Acceleration: the max change in speed every 80 ms.  0 means no limit.
        /// Units depend on your settings.
        /// Range = 0-255.
        /// Increment = 1.
        uint8_t acceleration;
    };

/// Represents the current status of a channel.
#pragma pack(push, 1)
    struct ServoStatus {
        /// The position in units of quarter-microseconds.
        uint16_t position;

        /// The target position in units of quarter-microseconds.
        uint16_t target;

        /// The speed limit.  Units depends on your settings.
        uint16_t speed;

        /// The acceleration limit.  Units depend on your settings.
        uint8_t acceleration;
    };
#pragma pack(pop)

    ~Device();

    const std::string &getName() const { return m_name; }
    int getNumChannels() const { return m_channelcnt; }

    /**
     * @brief Sets the target of the servo on channelNumber.
     *
     * If the channel is configured as a servo, then the target represents the
     * pulse width to transmit in units of quarter-microseconds. A \a target
     * value of 0 tells the Maestro to stop sending pulses to the servo.
     *
     * If the channel is configured as a digital output, values less than 6000
     * tell the Maestro to drive the line low, while values of 6000 or greater
     * tell the Maestro to drive the line high.
     *
     * @param channelNumber A servo number from 0 to 127.
     * @param target        A number from 0 to 16383.
     */
    void setTarget(uint8_t channelNumber, uint16_t target);

    /**
     * @brief Sets the \a speed limit of \a channelNumber.
     *
     * Limits the speed a servo channel’s output value changes.
     *
     * @param channelNumber A servo number from 0 to 127.
     * @param speed A number from 0 to 16383.
     */
    void setSpeed(uint8_t channelNumber, uint16_t target);

    /**
     * @brief Sets the \a acceleration limit of \a channelNumber.
     *
     * Limits the acceleration a servo channel’s output value changes.
     *
     * @param channelNumber A servo number from 0 to 127.
     * @param acceleration A number from 0 to 16383.
     */
    void setAcceleration(uint8_t channelNumber, uint16_t target);

    std::vector<ServoStatus> getServoStatus();

    void restoreDefaultConfiguration();

    DeviceSettings getDeviceSettings();
    void setDeviceSettings(const DeviceSettings &settings);

    ChannelSettings getChannelSettings(uint8_t channel);
    void setChannelSettings(uint8_t channel, const ChannelSettings &settings);

    void eraseScript();

    /**
     * @brief Starts loaded script at specified \a subroutineNumber location.
     *
     * Starts the loaded script at location specified by the subroutine number.
     * Subroutines are numbered in the order they are defined in loaded script.
     * Click the "View Compiled Code..." button and look at the subroutine list
     * to find the number for a particular subroutine.
     *
     * @param subroutineNumber A subroutine number defined in script's compiled code.
     */
    void restartScriptAtSubroutine(uint8_t subroutineNumber);

    /**
     * @brief Starts loaded script at specified \a subroutineNumber location after loading \a parameter on to the stack.
     *
     * Similar to the \p restartScript function, except it loads the parameter
     * on to the stack before starting the script at the specified subroutine
     * number location.
     *
     * @param subroutineNumber A subroutine number defined in script's compiled code.
     * @param parameter A number from 0 to 16383.     */
    void restartScriptAtSubroutineWithParameter(uint8_t subroutineNumber, uint16_t parameter);
    void restartScript();
    void setScriptDone(uint8_t value);
    void startBootloader();
    void reinitialize();
    void clearErrors();
    void writeScript(const std::vector<uint8_t> &bytecode);

    /**
     * @brief Sets the PWM specified by \a onTime and \a period in units of 1/48 microseconds.
     *
     * Sets the PWM output to the specified dutyCycle and period, in units of 1/48
     * microseconds.
     *
     * @param dutyCycle A number from 0 to 16320.
     * @param period A number from 4 to 16384.
     */
    void setPWM(uint16_t dutyCycle, uint16_t period);
    void disablePWM();

    /// Enumerate the Maestro devices.
    static std::vector<Device> getConnectedDevices();

   private:
    class usb_device;
    Device(usb_device *device, uint16_t productID);

    uint16_t getRawParameter(Parameter parameter);
    void setRawParameter(Parameter parameter, uint16_t value);
    void setRawParameterNoChecks(uint16_t parameter, uint16_t value, int bytes);

    const uint16_t m_vendorID = 0x1ffb;
    const uint16_t m_productID;
    std::string m_name;
    int m_channelcnt;

    std::shared_ptr<usb_device> m_dev = nullptr;
};
}  // namespace Maestro
