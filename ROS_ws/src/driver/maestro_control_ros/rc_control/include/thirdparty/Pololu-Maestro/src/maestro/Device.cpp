#include "Device.h"

#include <libusb.h>

#include <array>
#include <stdexcept>
#include <string>

// microsoft.....
#ifdef IGNORE
#undef IGNORE
#endif

namespace Maestro {
enum Device::Parameter : uint8_t {
    PARAMETER_INITIALIZED = 0,       // 1 byte - 0 or 0xFF
    PARAMETER_SERVOS_AVAILABLE = 1,  // 1 byte - 0-5
    PARAMETER_SERVO_PERIOD = 2,      // 1 byte - ticks allocated to each servo/256
    PARAMETER_SERIAL_MODE = 3,       // 1 byte unsigned value. Valid values are
    // SERIAL_MODE_*. Init variable.
    PARAMETER_SERIAL_FIXED_BAUD_RATE = 4,    // 2-byte unsigned value; 0 means autodetect. Init parameter.
    PARAMETER_SERIAL_TIMEOUT = 6,            // 2-byte unsigned value
    PARAMETER_SERIAL_ENABLE_CRC = 8,         // 1 byte boolean value
    PARAMETER_SERIAL_NEVER_SUSPEND = 9,      // 1 byte boolean value
    PARAMETER_SERIAL_DEVICE_NUMBER = 10,     // 1 byte unsigned value, 0-127
    PARAMETER_SERIAL_BAUD_DETECT_TYPE = 11,  // 1 byte value

    PARAMETER_IO_MASK_C = 16,      // 1 byte - pins used for I/O instead of servo
    PARAMETER_OUTPUT_MASK_C = 17,  // 1 byte - outputs that are enabled

    PARAMETER_CHANNEL_MODES_0_3 = 12,            // 1 byte - channel modes 0-3
    PARAMETER_CHANNEL_MODES_4_7 = 13,            // 1 byte - channel modes 4-7
    PARAMETER_CHANNEL_MODES_8_11 = 14,           // 1 byte - channel modes 8-11
    PARAMETER_CHANNEL_MODES_12_15 = 15,          // 1 byte - channel modes 12-15
    PARAMETER_CHANNEL_MODES_16_19 = 16,          // 1 byte - channel modes 16-19
    PARAMETER_CHANNEL_MODES_20_23 = 17,          // 1 byte - channel modes 20-23
    PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L = 18,  // servo period: 3-byte unsigned values, units of
    // quarter microseconds
    PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU = 19,
    PARAMETER_ENABLE_PULLUPS = 21,  // 1 byte: 0 or 1
    PARAMETER_SCRIPT_CRC = 22,      // 2 bytes - stores a checksum of the bytecode
    // program, for comparison
    PARAMETER_SCRIPT_DONE = 24,             // 1 byte - copied to scriptDone on startup
    PARAMETER_SERIAL_MINI_SSC_OFFSET = 25,  // 1 byte (0-254)
    PARAMETER_SERVO_MULTIPLIER = 26,        // 1 byte (0-255)

    // 9 * 24 = 216, so we can safely start at 30
    PARAMETER_SERVO0_HOME = 30,          // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO0_MIN = 32,           // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO0_MAX = 33,           // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO0_NEUTRAL = 34,       // 2 byte neutral position
    PARAMETER_SERVO0_RANGE = 36,         // 1 byte range
    PARAMETER_SERVO0_SPEED = 37,         // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO0_ACCELERATION = 38,  // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO1_HOME = 39,          // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO1_MIN = 41,           // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO1_MAX = 42,           // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO1_NEUTRAL = 43,       // 2 byte neutral position
    PARAMETER_SERVO1_RANGE = 45,         // 1 byte range
    PARAMETER_SERVO1_SPEED = 46,         // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO1_ACCELERATION = 47,  // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO2_HOME = 48,          // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO2_MIN = 50,           // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO2_MAX = 51,           // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO2_NEUTRAL = 52,       // 2 byte neutral position
    PARAMETER_SERVO2_RANGE = 54,         // 1 byte range
    PARAMETER_SERVO2_SPEED = 55,         // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO2_ACCELERATION = 56,  // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO3_HOME = 57,          // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO3_MIN = 59,           // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO3_MAX = 60,           // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO3_NEUTRAL = 61,       // 2 byte neutral position
    PARAMETER_SERVO3_RANGE = 63,         // 1 byte range
    PARAMETER_SERVO3_SPEED = 64,         // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO3_ACCELERATION = 65,  // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO4_HOME = 66,          // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO4_MIN = 68,           // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO4_MAX = 69,           // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO4_NEUTRAL = 70,       // 2 byte neutral position
    PARAMETER_SERVO4_RANGE = 72,         // 1 byte range
    PARAMETER_SERVO4_SPEED = 73,         // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO4_ACCELERATION = 74,  // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO5_HOME = 75,          // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO5_MIN = 77,           // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO5_MAX = 78,           // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO5_NEUTRAL = 79,       // 2 byte neutral position
    PARAMETER_SERVO5_RANGE = 81,         // 1 byte range
    PARAMETER_SERVO5_SPEED = 82,         // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO5_ACCELERATION = 83,  // 1 byte (speed changes that much every 10ms)
};

enum libusc_request : uint8_t {
    REQUEST_GET_PARAMETER = 0x81,
    REQUEST_SET_PARAMETER = 0x82,
    REQUEST_GET_VARIABLES = 0x83,
    REQUEST_SET_SERVO_VARIABLE = 0x84,
    REQUEST_SET_TARGET = 0x85,
    REQUEST_CLEAR_ERRORS = 0x86,
    REQUEST_GET_SERVO_SETTINGS = 0x87,

    // GET STACK and GET CALL STACK are only used on the Mini Maestro.
    REQUEST_GET_STACK = 0x88,
    REQUEST_GET_CALL_STACK = 0x89,
    REQUEST_SET_PWM = 0x8A,

    REQUEST_REINITIALIZE = 0x90,
    REQUEST_ERASE_SCRIPT = 0xA0,
    REQUEST_WRITE_SCRIPT = 0xA1,
    REQUEST_SET_SCRIPT_DONE = 0xA2,  // value.low.b is 0 for go, 1 for stop, 2 for single-step
    REQUEST_RESTART_SCRIPT_AT_SUBROUTINE = 0xA3,
    REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA4,
    REQUEST_RESTART_SCRIPT = 0xA5,
    REQUEST_START_BOOTLOADER = 0xFF,
};

struct Range {
    uint8_t bytes;
    int minimumValue;
    int maximumValue;
};

Range getRange(Device::Parameter parameterId) {
    // const libusc_range u32 = {4, 0, 0x7FFFFFFF};
    const Range u16 = {2, 0, 0xFFFF};
    // const libusc_range u12 = {2, 0, 0x0FFF};
    // const libusc_range u10 = {2, 0, 0x03FF};
    const Range u8 = {1, 0, 0xFF};
    const Range u7 = {1, 0, 0x7F};
    const Range boolean = {1, 0, 1};
    const Range offset = {1, 0, 254};
    const Range u2 = {1, 0, 3};
    const Range u32440 = {2, 0, 32440};
    const Range u50 = {1, 1, 50};

    switch (parameterId) {
        case Device::PARAMETER_INITIALIZED:
        case Device::PARAMETER_SERVOS_AVAILABLE:
        case Device::PARAMETER_SERVO_PERIOD:
        case Device::PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L:
        case Device::PARAMETER_SERVO_MULTIPLIER:
        case Device::PARAMETER_CHANNEL_MODES_0_3:
        case Device::PARAMETER_CHANNEL_MODES_4_7:
        case Device::PARAMETER_CHANNEL_MODES_8_11:
        case Device::PARAMETER_CHANNEL_MODES_12_15:
        case Device::PARAMETER_CHANNEL_MODES_16_19:
        case Device::PARAMETER_CHANNEL_MODES_20_23:
        case Device::PARAMETER_ENABLE_PULLUPS:
            return u8;
        case Device::PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU:
        case Device::PARAMETER_SERIAL_TIMEOUT:
        case Device::PARAMETER_SERIAL_FIXED_BAUD_RATE:
        case Device::PARAMETER_SCRIPT_CRC:
            return u16;
        case Device::PARAMETER_SERIAL_MODE:
            return u2;
        case Device::PARAMETER_SERIAL_BAUD_DETECT_TYPE:
        case Device::PARAMETER_SERIAL_NEVER_SUSPEND:
        case Device::PARAMETER_SERIAL_ENABLE_CRC:
        case Device::PARAMETER_SCRIPT_DONE:
            return boolean;
        case Device::PARAMETER_SERIAL_DEVICE_NUMBER:
            return u7;
        case Device::PARAMETER_SERIAL_MINI_SSC_OFFSET:
            return offset;
        default:
            break;
    }
    // must be one of the servo parameters
    switch ((((uint8_t)parameterId - (uint8_t)Device::PARAMETER_SERVO0_HOME) % 9) + (uint8_t)Device::PARAMETER_SERVO0_HOME) {
        case Device::PARAMETER_SERVO0_HOME:
        case Device::PARAMETER_SERVO0_NEUTRAL:
            return u32440;  // 32640 - 200
        case Device::PARAMETER_SERVO0_RANGE:
            return u50;  // the upper limit could be adjusted
        case Device::PARAMETER_SERVO0_SPEED:
        case Device::PARAMETER_SERVO0_MAX:
        case Device::PARAMETER_SERVO0_MIN:
        case Device::PARAMETER_SERVO0_ACCELERATION:
            return u8;
    }

    return u8;
}

Device::Parameter specifyServo(Device::Parameter p, uint8_t servo) {
    const uint8_t servoParameterBytes = 9;

    return Device::Parameter(uint8_t(p) + servo * servoParameterBytes);
}

uint16_t exponentialSpeedToNormalSpeed(uint8_t exponentialSpeed) {
    // Maximum value of normalSpeed is 31*(1<<7)=3968
    const int mantissa = exponentialSpeed >> 3;
    const int exponent = exponentialSpeed & 7;

    return uint16_t(mantissa * (1 << exponent));
}

uint8_t normalSpeedToExponentialSpeed(uint16_t normalSpeed) {
    uint16_t mantissa = normalSpeed;
    uint8_t exponent = 0;

    while (1) {
        if (mantissa < 32) {
            // We have reached the correct representation.
            return (uint8_t)(exponent + (mantissa << 3));
        }

        if (exponent == 7) {
            // The number is too big to express in this format.
            return 0xFF;
        }

        // Try representing the number with a bigger exponent.
        exponent += 1;
        mantissa >>= 1;
    }
}

/// See Sec 16.3 of the PIC18F14K50 datasheet for information about SPBRG.
/// On the umc01a, we have SYNC=0, BRG16=1, and BRGH=1, so the pure math
/// formula for the baud rate is Baud = INSTRUCTION_FREQUENCY / (spbrg+1);
uint32_t convertSpbrgToBps(uint16_t spbrg) {
    /// Instructions are executed at 12 MHZ
    const int INSTRUCTION_FREQUENCY = 12000000;

    if (spbrg == 0) {
        return 0;
    }

    return uint32_t(INSTRUCTION_FREQUENCY + (spbrg + 1) / 2) / (spbrg + 1);
}

/// The converts from bps to SPBRG, so it is the opposite of
/// libusc_convert_spbrg_to_bps. The purse math formula is spbrg =
/// INSTRUCTION_FREQUENCY/Baud - 1.
uint16_t convertBpsToSpbrg(uint32_t bps) {
    /// Instructions are executed at 12 MHZ
    const int INSTRUCTION_FREQUENCY = 12000000;

    if (bps == 0) {
        return 0;
    }

    return uint16_t((INSTRUCTION_FREQUENCY - bps / 2) / bps);
}

/// Converts channel number (0-5) to port mask bit number
/// on the Micro Maestro.  Not useful on other Maestros.
/// </summary>
uint8_t channelToPort(uint8_t channel) {
    if (channel <= 3) {
        return channel;
    } else if (channel < 6) {
        return channel + 2;
    }
    throw "Invalid channel number " + std::to_string(channel);
}

void requireArgumentRange(uint32_t argumentValue, uint32_t minimum, uint32_t maximum, std::string argumentName) {
    if (argumentValue < minimum || argumentValue > maximum) {
        throw "The " + argumentName + " must be between " + std::to_string(minimum) + " and " + std::to_string(maximum) +
            " but the value given was " + std::to_string(argumentValue);
    }
}

class Device::usb_device {
   public:
    usb_device(std::shared_ptr<libusb_context> context, libusb_device* device) : m_context(context), m_device(device) { libusb_ref_device(m_device); }
    ~usb_device() {
        close();
        libusb_unref_device(m_device);
    }
    void open() {
        if (!m_deviceHandle) {
            libusb_open(m_device, &m_deviceHandle);
        }
    }
    void close() { libusb_close(m_deviceHandle); }

    uint32_t controlTransfer(uint8_t RequestType, uint8_t Request, uint16_t Value, uint16_t Index, uint8_t* data = nullptr, uint16_t length = 0) {
        open();

        const int ret = libusb_control_transfer(m_deviceHandle, RequestType, Request, Value, Index, data, length, 5000);
        switch (ret) {
            case LIBUSB_ERROR_TIMEOUT:
                throw "the transfer timed out";
            case LIBUSB_ERROR_PIPE:
                throw "the control request was not supported by the device";
            case LIBUSB_ERROR_NO_DEVICE:
                throw "the device has been disconnected";
            case LIBUSB_ERROR_BUSY:
                throw "called from event handling context";
            case LIBUSB_ERROR_INVALID_PARAM:
                throw "the transfer size is larger than the operating system and/or hardware can support";
            default:
                break;
        }
        return ret;
    }

   private:
    std::shared_ptr<libusb_context> m_context = nullptr;
    libusb_device* m_device = nullptr;
    libusb_device_handle* m_deviceHandle = nullptr;
};

std::vector<Device> Device::getConnectedDevices() {
    const uint16_t vendorID = 0x1ffb;
    const std::array<uint16_t, 4> productIDArray = {0x0089, 0x008a, 0x008b, 0x008c};

    std::shared_ptr<libusb_context> sharedContext(nullptr, [=](libusb_context* ctx) { libusb_exit(ctx); });

    libusb_context* ctx = sharedContext.get();
    libusb_init(&ctx);

    std::vector<Device> list;

    libusb_device** devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) return list;

    for (int i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        const int r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) {
            fprintf(stderr, "failed to get device descriptor");
            continue;
        }

        if (desc.idVendor == vendorID) {
            for (int productID : productIDArray) {
                if (desc.idProduct == productID) {
                    usb_device* device = new usb_device(sharedContext, devs[i]);
                    list.push_back(Device(device, desc.idProduct));
                }
            }
        }
    }
    libusb_free_device_list(devs, 1);

    return list;
}

Device::Device(usb_device* device, uint16_t productId) : m_productID(productId), m_dev(device) {
    switch (m_productID) {
        case 0x89:
            m_channelcnt = 6;
            m_name = "Micro Maestro 6";
            break;
        case 0x8A:
            m_channelcnt = 12;
            m_name = "Mini Maestro 12";
            break;
        case 0x8B:
            m_channelcnt = 18;
            m_name = "Mini Maestro 18";
            break;
        case 0x8C:
            m_channelcnt = 24;
            m_name = "Mini Maestro 24";
            break;
        default:
            throw "Unknown product id " + std::to_string(m_productID);
    }
}

Device::~Device() {}

void Device::setTarget(uint8_t servo, uint16_t value) {
    try {
        m_dev->controlTransfer(0x40, REQUEST_SET_TARGET, value, servo);
    } catch (std::exception& e) {
        throw "Failed to set target of servo " + std::to_string(servo) + " to " + std::to_string(value) + ".";
    }
}

void Device::setSpeed(uint8_t servo, uint16_t value) {
    try {
        m_dev->controlTransfer(0x40, REQUEST_SET_SERVO_VARIABLE, value, servo);
    } catch (std::exception& e) {
        throw "Failed to set speed of servo " + std::to_string(servo) + " to " + std::to_string(value) + ".";
    }
}

void Device::setAcceleration(uint8_t servo, uint16_t value) {
    // set the high bit of servo to specify acceleration
    try {
        m_dev->controlTransfer(0x40, REQUEST_SET_SERVO_VARIABLE, value, servo | 0x80);
    } catch (std::exception& e) {
        throw "Failed to set acceleration of servo " + std::to_string(servo) + " to " + std::to_string(value) + ".";
    }
}

std::vector<Device::ServoStatus> Device::getServoStatus() {
    static_assert(sizeof(ServoStatus) == 7, "Sizeof ServoStatus expected to be 7");

    const uint32_t size = m_channelcnt * sizeof(ServoStatus);
    std::vector<ServoStatus> status(m_channelcnt);

    const uint32_t bytesRead = m_dev->controlTransfer(0xC0, REQUEST_GET_SERVO_SETTINGS, 0, 0, (uint8_t*)status.data(), size);

    if (bytesRead != size) {
        throw "Short read: " + std::to_string(bytesRead) + " < " + std::to_string(size) + ".";
    }
    return status;
}

void Device::restoreDefaultConfiguration() {
    setRawParameterNoChecks(PARAMETER_INITIALIZED, 0xFF, 1);
    reinitialize();
}

Device::DeviceSettings Device::getDeviceSettings() {
    uint8_t buffer[14];
    m_dev->controlTransfer(0x80, 6, 0x0100, 0x0000, buffer, 14);

    DeviceSettings settings;
    settings.firmwareVersionMinor = uint8_t((buffer[12] & 0xF) + (buffer[12] >> 4 & 0xF) * 10);
    settings.firmwareVersionMajor = uint8_t((buffer[13] & 0xF) + (buffer[13] >> 4 & 0xF) * 10);

    settings.serialMode = (SerialMode)getRawParameter(PARAMETER_SERIAL_MODE);
    settings.fixedBaudRate = convertSpbrgToBps(getRawParameter(PARAMETER_SERIAL_FIXED_BAUD_RATE));
    settings.enableCrc = getRawParameter(PARAMETER_SERIAL_ENABLE_CRC) != 0;
    settings.neverSuspend = getRawParameter(PARAMETER_SERIAL_NEVER_SUSPEND) != 0;
    settings.serialDeviceNumber = getRawParameter(PARAMETER_SERIAL_DEVICE_NUMBER);
    settings.miniSscOffset = getRawParameter(PARAMETER_SERIAL_MINI_SSC_OFFSET);
    settings.serialTimeout = getRawParameter(PARAMETER_SERIAL_TIMEOUT);
    settings.scriptDone = getRawParameter(PARAMETER_SCRIPT_DONE) != 0;

    if (m_channelcnt == 6) {
        settings.servosAvailable = getRawParameter(PARAMETER_SERVOS_AVAILABLE);
        settings.servoPeriod = getRawParameter(PARAMETER_SERVO_PERIOD);
    } else {
        uint32_t tmp = getRawParameter(PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU) << 8;
        tmp |= (uint8_t)getRawParameter(PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L);
        settings.miniMaestroServoPeriod = tmp;

        settings.servoMultiplier = getRawParameter(PARAMETER_SERVO_MULTIPLIER) + 1;
    }

    if (m_channelcnt > 18) {
        settings.enablePullups = getRawParameter(PARAMETER_ENABLE_PULLUPS) != 0;
    }
    return settings;
}

void Device::setDeviceSettings(const DeviceSettings& settings) {
    setRawParameter(PARAMETER_SERIAL_MODE, (uint8_t)settings.serialMode);
    setRawParameter(PARAMETER_SERIAL_FIXED_BAUD_RATE, convertBpsToSpbrg(settings.fixedBaudRate));
    setRawParameter(PARAMETER_SERIAL_ENABLE_CRC, settings.enableCrc ? 1 : 0);
    setRawParameter(PARAMETER_SERIAL_NEVER_SUSPEND, settings.neverSuspend ? 1 : 0);
    setRawParameter(PARAMETER_SERIAL_DEVICE_NUMBER, settings.serialDeviceNumber);
    setRawParameter(PARAMETER_SERIAL_MINI_SSC_OFFSET, settings.miniSscOffset);
    setRawParameter(PARAMETER_SERIAL_TIMEOUT, settings.serialTimeout);
    setRawParameter(PARAMETER_SCRIPT_DONE, uint16_t(settings.scriptDone ? 1 : 0));

    if (m_channelcnt == 6) {
        setRawParameter(PARAMETER_SERVOS_AVAILABLE, settings.servosAvailable);
        setRawParameter(PARAMETER_SERVO_PERIOD, settings.servoPeriod);
    } else {
        setRawParameter(PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L, uint8_t(settings.miniMaestroServoPeriod & 0xFF));
        setRawParameter(PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU, uint16_t(settings.miniMaestroServoPeriod >> 8));

        uint8_t multiplier;
        if (settings.servoMultiplier < 1) {
            multiplier = 0;
        } else if (settings.servoMultiplier > 256) {
            multiplier = 255;
        } else {
            multiplier = settings.servoMultiplier - 1;
        }
        setRawParameter(PARAMETER_SERVO_MULTIPLIER, multiplier);
    }

    if (m_channelcnt > 18) {
        setRawParameter(PARAMETER_ENABLE_PULLUPS, settings.enablePullups ? 1 : 0);
    }
}

Device::ChannelSettings Device::getChannelSettings(uint8_t channel) {
    ChannelSettings settings;
    if (m_channelcnt == 6) {
        uint8_t ioMask = (uint8_t)getRawParameter(PARAMETER_IO_MASK_C);
        uint8_t outputMask = (uint8_t)getRawParameter(PARAMETER_OUTPUT_MASK_C);
        uint8_t bitmask = uint8_t(1 << channelToPort(channel));
        if ((ioMask & bitmask) == 0) {
            settings.mode = ChannelMode::SERVO;
        } else if ((outputMask & bitmask) == 0) {
            settings.mode = ChannelMode::INPUT;
        } else {
            settings.mode = ChannelMode::OUTPUT;
        }
    } else {
        uint8_t channelModeBytes = (uint8_t)getRawParameter(Parameter(PARAMETER_CHANNEL_MODES_0_3 + (channel >> 2)));

        settings.mode = ChannelMode((channelModeBytes >> ((channel & 3) << 1)) & 3);
    }

    const uint16_t home = getRawParameter(specifyServo(PARAMETER_SERVO0_HOME, channel));
    if (home == 0) {
        settings.homeMode = HomeMode::OFF;
        settings.home = 0;
    } else if (home == 1) {
        settings.homeMode = HomeMode::IGNORE;
        settings.home = 0;
    } else {
        settings.homeMode = HomeMode::GOTO;
        settings.home = home;
    }

    settings.minimum = 64 * getRawParameter(specifyServo(PARAMETER_SERVO0_MIN, channel));
    settings.maximum = 64 * getRawParameter(specifyServo(PARAMETER_SERVO0_MAX, channel));
    settings.neutral = getRawParameter(specifyServo(PARAMETER_SERVO0_NEUTRAL, channel));
    settings.range = 127 * getRawParameter(specifyServo(PARAMETER_SERVO0_RANGE, channel));
    settings.speed = exponentialSpeedToNormalSpeed((uint8_t)getRawParameter(specifyServo(PARAMETER_SERVO0_SPEED, channel)));
    settings.acceleration = (uint8_t)getRawParameter(specifyServo(PARAMETER_SERVO0_ACCELERATION, channel));
    return settings;
}

void Device::setChannelSettings(uint8_t channel, const ChannelSettings& settings) {
    if (m_channelcnt == 6) {
        /*
        if (settings.mode == ChannelMode::INPUT || settings.mode == ChannelMode::OUTPUT) {
            ioMask |= (uint8_t)(1 << channelToPort(channel));
        }

        if (setting.mode == ChannelMode.Output) {
            outputMask |= (uint8_t)(1 << channelToPort(channel));
        }*/
    } else {
        Parameter parameter = Parameter(PARAMETER_CHANNEL_MODES_0_3 + (channel >> 2));
        uint8_t channelModeBytes = (uint8_t)getRawParameter(parameter);

        channelModeBytes &= (uint8_t) ~(3 << ((channel & 3) << 1));
        channelModeBytes |= uint8_t(uint8_t(settings.mode) << ((channel & 3) << 1));

        setRawParameter(parameter, channelModeBytes);
    }

    // Make sure that HomeMode is "Ignore" for inputs.
    HomeMode correctedHomeMode = settings.homeMode;
    if (settings.mode == ChannelMode::INPUT) {
        correctedHomeMode = HomeMode::IGNORE;
    }

    setRawParameter(specifyServo(PARAMETER_SERVO0_HOME, channel), uint8_t(correctedHomeMode));

    setRawParameter(specifyServo(PARAMETER_SERVO0_MIN, channel), settings.minimum / 64);
    setRawParameter(specifyServo(PARAMETER_SERVO0_MAX, channel), settings.maximum / 64);
    setRawParameter(specifyServo(PARAMETER_SERVO0_NEUTRAL, channel), settings.neutral);
    setRawParameter(specifyServo(PARAMETER_SERVO0_RANGE, channel), settings.range / 127);
    setRawParameter(specifyServo(PARAMETER_SERVO0_SPEED, channel), normalSpeedToExponentialSpeed(settings.speed));
    setRawParameter(specifyServo(PARAMETER_SERVO0_ACCELERATION, channel), settings.acceleration);

    if (m_channelcnt == 6) { /*
                                setRawParameter(PARAMETER_IO_MASK_C, ioMask);
                                setRawParameter(PARAMETER_OUTPUT_MASK_C, outputMask);*/
    } else {                 /*
                                for (uint8_t i = 0; i < 6; i++)
                                {
                                    setRawParameter(PARAMETER_CHANNEL_MODES_0_3 + i, channelModeBytes[i]);
                                }*/
    }
}

/// Erases the entire script and subroutine address table from the devices.
void Device::eraseScript() {
    try {
        m_dev->controlTransfer(0x40, REQUEST_ERASE_SCRIPT, 0, 0);
    } catch (std::exception& e) {
        throw "There was an error erasing the script.";
    }
}

void Device::restartScriptAtSubroutine(uint8_t subroutine) {
    try {
        m_dev->controlTransfer(0x40, REQUEST_RESTART_SCRIPT_AT_SUBROUTINE, 0, subroutine);
    } catch (std::exception& e) {
        throw "There was an error restarting the script at subroutine " + std::to_string(subroutine) + ".";
    }
}

void Device::restartScriptAtSubroutineWithParameter(uint8_t subroutine, uint16_t parameter) {
    try {
        m_dev->controlTransfer(0x40, REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER, parameter, subroutine);
    } catch (std::exception& e) {
        throw "There was an error restarting the script with a parameter at subroutine " + std::to_string(subroutine) + ".";
    }
}

void Device::restartScript() {
    try {
        m_dev->controlTransfer(0x40, REQUEST_RESTART_SCRIPT, 0, 0);
    } catch (std::exception& e) {
        throw "There was an error restarting the script.";
    }
}

void Device::setScriptDone(uint8_t value) {
    try {
        m_dev->controlTransfer(0x40, REQUEST_SET_SCRIPT_DONE, value, 0);
    } catch (std::exception& e) {
        throw "There was an error setting the script done.";
    }
}

void Device::startBootloader() {
    try {
        m_dev->controlTransfer(0x40, REQUEST_START_BOOTLOADER, 0, 0);
    } catch (std::exception& e) {
        throw "There was an error entering bootloader mode.";
    }
}

void Device::reinitialize() {
    try {
        m_dev->controlTransfer(0x40, REQUEST_REINITIALIZE, 0, 0);
    } catch (std::exception& e) {
        throw "There was an error re-initializing the device.";
    }
}

void Device::clearErrors() {
    try {
        m_dev->controlTransfer(0x40, REQUEST_CLEAR_ERRORS, 0, 0);
    } catch (std::exception& e) {
        throw "There was a USB communication error while clearing the servo errors.";
    }
}

void Device::writeScript(const std::vector<uint8_t>& bytecode) {
    for (uint16_t block = 0; block < (bytecode.size() + 15) / 16; block++) {
        // write each block in a separate request
        uint8_t block_bytes[16];

        for (uint16_t j = 0; j < 16; j++) {
            if (block * 16 + j < bytecode.size())
                block_bytes[j] = bytecode[block * 16 + j];
            else
                block_bytes[j] = (uint8_t)0xFF;  // don't change flash if it is not necessary
        }

        try {
            m_dev->controlTransfer(0x40, REQUEST_WRITE_SCRIPT, 0, block, block_bytes);
        } catch (std::exception& e) {
            throw "There was an error writing script block " + std::to_string(block) + ".";
        }
    }
}

void Device::setPWM(uint16_t dutyCycle, uint16_t period) { m_dev->controlTransfer(0x40, REQUEST_SET_PWM, dutyCycle, period); }

void Device::disablePWM() {
    if (m_productID == 0x008a)
        setTarget(8, 0);
    else
        setTarget(12, 0);
}

uint16_t Device::getRawParameter(Parameter parameter) {
    const Range range = getRange(parameter);
    uint16_t value = 0;
    uint16_t buffer;

    try {
        m_dev->controlTransfer(0xC0, REQUEST_GET_PARAMETER, 0, parameter, (uint8_t*)&buffer, range.bytes);
    } catch (std::exception& e) {
        throw "There was an error getting parameter from the device.";
    }

    if (range.bytes == 1) {
        // read a single byte
        value = *(uint8_t*)&buffer;
    } else {
        // read two bytes
        value = buffer;
    }

    return value;
}

void Device::setRawParameter(Parameter parameter, uint16_t value) {
    const Range range = getRange(parameter);
    // requireArgumentRange(value, range.minimumValue, range.maximumValue,
    // parameter.ToString());
    const int bytes = range.bytes;
    setRawParameterNoChecks(parameter, value, bytes);
}

void Device::setRawParameterNoChecks(uint16_t parameter, uint16_t value, int bytes) {
    uint16_t index = (uint16_t)((bytes << 8) + parameter);  // high bytes = # of bytes
    try {
        m_dev->controlTransfer(0x40, REQUEST_SET_PARAMETER, value, index);
    } catch (std::exception& e) {
        throw "There was an error setting parameter on the device.";
    }
}
}  // namespace Maestro
