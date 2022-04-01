# tcan_can_j1919

## Package structure
This package contains
* classes to hlep with decoding J1939 messages
* messages, deriving from `J1939PgnParser`, as defined in the J1939 Digital Annex (RSL members can search on GDrive for `J1939DA_201907.xls` to find a copy, otherwise obtain a copy from https://www.sae.org/standards/content/j1939da_201907/)
* devices combining multiple messages, as they were purchased. These should be used in your code and can be added to `tcan_can::CanManager`s

## Messages
Messages inherit from `J1939PgnParser`, and provide a minimal implementation, as defined in the J1939 Digital Annex. 

To get a copy,
* RSL members can search for `J1939DA_201907.xls` on GDrive
* the standard can be purchased at https://www.sae.org/standards/content/j1939da_201907/

Messages should return their values in the unit defined in the standard. Units should be documented near the variable.

## Devices
Devices inherit from `DeviceJ1939`, and can be included in your `tcan_can::Canmanager`. They provide a collection of messages, as they are available on a device that was purchased.

Getters on devices should preferably return in SI units, unless you have a strong reason to prefer something else. Getters should provide documentation on the units used.

## Example classes for developers
If you add your messages, follow these guidelines

`include/tcan_can:j1939/messages/ExampleMsg.hpp`
```
// Find the following information in the digital annex, and adapt code accordingly
// MsgName
// PGN
// resolution & offset for each field
// offset

#pragma once

#include "tcan_can_j1939/J1939PgnParser.hpp"

namespace tcan_can_j1939 {
namespace messages {
struct ExampleMsg : public J1939PgnParser {
    ExampleMsg : J1939PgnParser(PGN) {}

    bool parse(const tcan_can::CanMsg& msg) {       
        field0_ = scaledMessageFromRaw(msg.readuint16(0), resolution, offset);      // Adapt according to message definition
        // Provide other fields as necessary
        return true;
    }

    double field0_{}; // some crazy unit, as specified in the Digital Annex
};
}  // namespace messages
}  // namespace tcan_can_j1939
```

`include/tcan_can_j1939/unit_conversions.hpp`
```
// ... other conversions

inline double normalFromCrazy(double crazy) {
    return crazy * x + y;
}

// ...other conversions
```

`include/tcan_can_j1939/devices/ExampleDevice.hpp`
```
#pragma once

#include "tcan_can_j1939/DeviceJ1939.hpp"
#include "tcan_can/unit_conversions.hpp"        // if needed
#include "tcan_can_j1939/messages/ExampleMsg.hpp"
#include "tcan_can_j1939/messages/OtherMessage.hpp"

namespace tcan_can_j1939 {
namespace devices {

class ExampleDevice : public DeviceJ1939 {
   public:
    template <typename... Args>
    Imu(Args&&... args) : DeviceJ1939(std::forward<Args>(args)...) {
        addParser(exampleMsg_);
        addParser(otherMsg_);

    }
    ~Imu() override = default;

    double getField0() const { return normalFromCrazy(exampleMsg_.field0_); } // normal unit
    // Add other msgs as required

   private:
    messages::ExampleMsg exampleMsg_;
    messages::OtherMsg otherMsg_;

};

}  // namespace devices
}  // namespace tcan_can_j1939
```

`src/Devices.cpp`
```
// Includes to other devices
#include "tcan_can_j1939/devices/ExampleDevice.hpp"
// Includes to other devices

// Leave empty, this file only exists to check for errors in the device and message definitions during compilation of this package
```
