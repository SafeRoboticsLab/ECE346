#include <maestro/Device.h>
#include <maestro/Program.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace Maestro;

// clang-format off
PYBIND11_MODULE(maestro, m)
{
    m.attr("__version__") = "0.1.0";
    m.def("getConnectedDevices", &Device::getConnectedDevices);

    py::class_<Device> device(m, "Device");

    py::enum_<Device::SerialMode>(device, "SerialMode")
        .value("SERIAL_MODE_USB_DUAL_PORT", Device::SerialMode::SERIAL_MODE_USB_DUAL_PORT)
        .value("SERIAL_MODE_USB_CHAINED", Device::SerialMode::SERIAL_MODE_USB_CHAINED)
        .value("SERIAL_MODE_UART_DETECT_BAUD_RATE", Device::SerialMode::SERIAL_MODE_UART_DETECT_BAUD_RATE)
        .value("SERIAL_MODE_UART_FIXED_BAUD_RATE", Device::SerialMode::SERIAL_MODE_UART_FIXED_BAUD_RATE)
        .export_values();

    py::class_<Device::DeviceSettings>(device, "DeviceSettings")
        .def_readwrite("firmwareVersionMinor", &Device::DeviceSettings::firmwareVersionMinor)
        .def_readwrite("firmwareVersionMajor", &Device::DeviceSettings::firmwareVersionMajor)
        .def_readwrite("servosAvailable", &Device::DeviceSettings::servosAvailable)
        .def_readwrite("servoPeriod", &Device::DeviceSettings::servoPeriod)
        .def_readwrite("miniMaestroServoPeriod", &Device::DeviceSettings::miniMaestroServoPeriod)
        .def_readwrite("servoMultiplier", &Device::DeviceSettings::servoMultiplier)
        .def_readwrite("fixedBaudRate", &Device::DeviceSettings::fixedBaudRate)
        .def_readwrite("enableCrc", &Device::DeviceSettings::enableCrc)
        .def_readwrite("neverSuspend", &Device::DeviceSettings::neverSuspend)
        .def_readwrite("serialDeviceNumber", &Device::DeviceSettings::serialDeviceNumber)
        .def_readwrite("miniSscOffset", &Device::DeviceSettings::miniSscOffset)
        .def_readwrite("serialTimeout", &Device::DeviceSettings::serialTimeout)
        .def_readwrite("scriptDone", &Device::DeviceSettings::scriptDone)
        .def_readwrite("enablePullups", &Device::DeviceSettings::enablePullups)
        ;

    py::enum_<Device::ChannelMode>(device, "ChannelMode")
        .value("SERVO", Device::ChannelMode::SERVO)
        .value("SERVO_MULTIPLIED", Device::ChannelMode::SERVO_MULTIPLIED)
        .value("OUTPUT", Device::ChannelMode::OUTPUT)
        .value("INPUT", Device::ChannelMode::INPUT)
        .export_values();

    py::enum_<Device::HomeMode>(device, "HomeMode")
        .value("OFF", Device::HomeMode::OFF)
        .value("IGNORE", Device::HomeMode::IGNORE)
        .value("GOTO", Device::HomeMode::GOTO)
        .export_values();

    py::class_<Device::ChannelSettings>(device, "ChannelSettings")
        .def_readwrite("mode", &Device::ChannelSettings::mode)
        .def_readwrite("homeMode", &Device::ChannelSettings::homeMode)
        .def_readwrite("home", &Device::ChannelSettings::home)
        .def_readwrite("minimum", &Device::ChannelSettings::minimum)
        .def_readwrite("maximum", &Device::ChannelSettings::maximum)
        .def_readwrite("neutral", &Device::ChannelSettings::neutral)
        .def_readwrite("range", &Device::ChannelSettings::range)
        .def_readwrite("speed", &Device::ChannelSettings::speed)
        .def_readwrite("acceleration", &Device::ChannelSettings::acceleration)
        ;

    py::class_<Device::ServoStatus>(device, "ServoStatus")
        .def_readonly("position", &Device::ServoStatus::position)
        .def_readonly("target", &Device::ServoStatus::target)
        .def_readonly("speed", &Device::ServoStatus::speed)
        .def_readonly("acceleration", &Device::ServoStatus::acceleration)
        ;

    device.def("getName", &Device::getName)
          .def("getNumChannels", &Device::getNumChannels)
          .def("setTarget", &Device::setTarget, py::arg("channelNumber"), py::arg("target"))
          .def("setSpeed", &Device::setSpeed, py::arg("channelNumber"), py::arg("target"))
          .def("setAcceleration", &Device::setAcceleration, py::arg("channelNumber"), py::arg("target"))
          .def("getServoStatus", &Device::getServoStatus)
          .def("restoreDefaultConfiguration", &Device::restoreDefaultConfiguration)
          .def("getDeviceSettings", &Device::getDeviceSettings)
          .def("setDeviceSettings", &Device::setDeviceSettings, py::arg("settings"))
          .def("getChannelSettings", &Device::getChannelSettings)
          .def("setChannelSettings", &Device::setChannelSettings, py::arg("channelNumber"), py::arg("settings"))
          .def("eraseScript", &Device::eraseScript)
          .def("restartScriptAtSubroutine", &Device::restartScriptAtSubroutine, py::arg("subroutineNumber"))
          .def("restartScriptAtSubroutineWithParameter", &Device::restartScriptAtSubroutineWithParameter, py::arg("subroutineNumber"), py::arg("parameter"))
          .def("restartScript", &Device::restartScript)
          .def("setScriptDone", &Device::setScriptDone, py::arg("value"))
          .def("startBootloader", &Device::startBootloader)
          .def("reinitialize", &Device::reinitialize)
          .def("clearErrors", &Device::clearErrors)
          .def("writeScript", &Device::writeScript, py::arg("bytecode"))
          .def("setPWM", &Device::setPWM, py::arg("dutyCycle"), py::arg("period"))
          .def("disablePWM", &Device::disablePWM)
          ;

    py::class_<Program>(m, "Program")
          .def(py::init<const std::string &, bool>(), py::arg("script"), py::arg("isMiniMaestro"))
          .def("getByteList", &Program::getByteList)
          .def("getCRC",  &Program::getCRC)
          .def("toString", &Program::toString);
}
