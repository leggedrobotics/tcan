#pragma once

#include <tcan_can/J1939PgnParser.hpp>

namespace tcan_can_j1939_devices {
struct AccelerationSensor : public tcan_can::J1939PgnParser {
    AccelerationSensor() : tcan_can::J1939PgnParser(0xF02D) {}

    bool parse(const tcan_can::CanMsg& msg) {
        lateralAcceleration_ = scaledMessageFromRaw(msg.readuint16(0), 0.01, -320.);
        longitudinalAcceleration_ = scaledMessageFromRaw(msg.readuint16(2), 0.01, -320.);
        verticalAcceleration = scaledMessageFromRaw(msg.readuint16(4), 0.01, -320.);
        return true;
    }

    double lateralAcceleration_{0.};
    double longitudinalAcceleration_{0.};
    double verticalAcceleration{0.};
};
}  // namespace tcan_can_j1939_devices