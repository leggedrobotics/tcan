/*!
 * @file 	JoystickUsb.cpp
 * @author 	Philipp Leemann
 * @date		June 9, 2015
 * @version 	0.0
 */


#include <fstream>

#include "tcan_usb_devices/JoystickUsb.hpp"

namespace tcan_usb_devices {

JoystickUsb::JoystickUsb(std::unique_ptr<JoystickUsbOptions>&& options):
    tcan_usb::UniversalSerialBus(std::move(options)),
    mutex_(),
    buttons_(),
    axes_()
{
}

void JoystickUsb::handleMessage(const tcan_usb::UsbMsg& msg) {
    if(msg.getLength() != sizeof(js_event)) {
        return;
    }

    js_event event;
    memcpy(&event, msg.getData(), sizeof(js_event));

    // Parameter conversions
    const double deadzone = static_cast<const JoystickUsbOptions*>(options_.get())->getDeadzone();
    const double scale = -1. / (1. - deadzone) / 32767.;
    double unscaled_deadzone = 32767. * deadzone;

    switch(event.type) {

        // Check buttons
        case JS_EVENT_BUTTON:
        case JS_EVENT_BUTTON | JS_EVENT_INIT:
        // increase size of button container if necessary
        if(event.number >= buttons_.size()) {
            std::lock_guard<std::mutex> lock(mutex_);

            int old_size = buttons_.size();
            buttons_.resize(event.number+1);

            // init with 0.0
            for(unsigned int i=old_size;i<buttons_.size();i++) {
                buttons_[i] = 0;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            buttons_[event.number] = (event.value ? 1 : 0);
        }
        break;

        // Check axes
        case JS_EVENT_AXIS:
        case JS_EVENT_AXIS | JS_EVENT_INIT:
        // increase size of axes container if necessary
        if(event.number >= axes_.size()) {
            std::lock_guard<std::mutex> lock(mutex_);

            int old_size = axes_.size();
            axes_.resize(event.number+1);

            // init with 0.0
            for(unsigned int i=old_size;i<axes_.size();i++) {
                axes_[i] = 0.0;
            }
        }
        if (!(event.type & JS_EVENT_INIT)) // value is wrong on Init event
        {
            double val = event.value;
            // Allows deadzone to be "smooth"
            if (val > unscaled_deadzone) {
                val -= unscaled_deadzone;
            }else if (val < -unscaled_deadzone) {
                val += unscaled_deadzone;
            }else{
                val = 0;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                axes_[event.number] = val * scale;
            }
        }

        break;

        default:
            printf("Unknown event type. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
            break;
    }
}

double JoystickUsb::getAxis(const unsigned int ind) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if(ind >= axes_.size()) {
        return 0.0;
    }

    return axes_[ind];
}

int JoystickUsb::getButton(const unsigned int ind) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if(ind >= buttons_.size()) {
        return 0;
    }

    return buttons_[ind];
}

unsigned int JoystickUsb::getNumAxis() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return axes_.size();
}

unsigned int JoystickUsb::getNumButtons() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buttons_.size();
}

bool JoystickUsb::getDevicesPaths(const char* vid, const char* pid, std::vector<std::string> &paths) {
    std::fstream file_vid, file_pid, file_dev;

    for(int i=0; i<32; i++) { // there are max. 32 joysticks
        // check if joystick exists
        std::ostringstream str_vid, str_pid;
        str_vid << "/sys/class/input/js" << i << "/device/id/vendor";
        str_pid << "/sys/class/input/js" << i << "/device/id/product";

        file_vid.open( str_vid.str(), std::ios_base::in );
        file_pid.open( str_pid.str(), std::ios_base::in );
        if(file_vid.is_open() && file_pid.is_open()) {
            // check if PID and VID are equal to given ids
            char v[5], p[5];
            file_vid.get(v, 5);
            file_pid.get(p, 5);
            if(strncmp(v, vid, 4) == 0 && strncmp(p, pid, 4) == 0) {
                // check if fd can be opened (without root)
                std::ostringstream path;
                path << "/dev/input/js" << i;
                file_dev.open( path.str(), std::ios_base::in );
                if(file_dev.is_open()) {
                    paths.push_back( path.str() );
                }else{
                    std::cout << "  Unable to open " << path.str() << pid << std::endl;
                }
                file_dev.close();
            }else{
//                std::cout << "  " << v << " != " << vid << " or " << p << " != " << pid << std::endl;
            }
        }else{
//            std::cout << "  File " << str_vid.str() << " or " << str_pid.str() << "does not exist" << std::endl;
        }
        file_vid.close();
        file_pid.close();
    }

    return true;
}

} // end namespace
