/*!
* @file     JoystickUsbOptions.hpp
* @author   Philipp Leemann
* @date     Nov, 1, 2015
* @brief
*/

#pragma once

#include <linux/joystick.h>

#include "tcan_usb/UniversalSerialBusOptions.hpp"


namespace tcan_usb_devices {

class JoystickUsbOptions : public tcan_usb::UniversalSerialBusOptions {
 public:
    JoystickUsbOptions() = delete;
    JoystickUsbOptions(const std::string& name, const double deadzone):
        tcan_usb::UniversalSerialBusOptions(name, sizeof(js_event))
    {
        skipConfiguration = true;
        maxDeviceTimeoutCounter = 0;
        setDeadzone(deadzone);
    }

    void setDeadzone(const double deadzone) noexcept {
        if (deadzone > 0.9) {
            printf("deadzone (%f) greater than 0.9, setting it to 0.9", deadzone);
            deadzone_ = 0.9;
        }else if(deadzone < 0.0) {
            printf("joy_node: deadzone (%f) less than 0, setting to 0.", deadzone);
            deadzone_ = 0.0;
        }
    }

    double getDeadzone() const noexcept { return deadzone_; }

 private:
    double deadzone_;
};

} // end namespace

