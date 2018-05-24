/*!
* @file     JoystickUsb.hpp
* @author   Philipp Leemann
* @date     June, 9, 2015
* @brief
*/

#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include "tcan_usb/UniversalSerialBus.hpp"
#include "tcan_usb_devices/JoystickUsbOptions.hpp"


namespace tcan_usb_devices {

class JoystickUsb : public tcan_usb::UniversalSerialBus {
 public:
    JoystickUsb() = delete;
    JoystickUsb(std::unique_ptr<JoystickUsbOptions>&& options);

    ~JoystickUsb() override = default;

    /*! Callback called after reception of a message.
     * @param msg   reference to the usb message
     */
    void handleMessage(const tcan_usb::UsbMsg& msg) override;

    double getAxis(const unsigned int ind) const;
    int getButton(const unsigned int ind) const;

    unsigned int getNumAxis() const;
    unsigned int getNumButtons() const;

    static bool getDevicesPaths(const char* vid, const char* pid, std::vector<std::string> &paths);

 private:
    mutable std::mutex mutex_;
    std::vector<int> buttons_;
    std::vector<double> axes_;
};

} // end namespace

