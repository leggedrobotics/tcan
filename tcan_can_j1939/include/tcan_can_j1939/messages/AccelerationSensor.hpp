#pragma once

#include "tcan_can_j1939/J1939PgnParser.hpp"

namespace tcan_can_j1939 {
namespace messages {
struct AccelerationSensor : public J1939PgnParser {
    AccelerationSensor() : J1939PgnParser(0xF02D) {}

    bool parse(const tcan_can::CanMsg& msg) override {
        lateralAcceleration_ = scaledMessageFromRaw(msg.readuint16(0), 0.01, -320.);
        longitudinalAcceleration_ = scaledMessageFromRaw(msg.readuint16(2), 0.01, -320.);
        verticalAcceleration = scaledMessageFromRaw(msg.readuint16(4), 0.01, -320.);
        return true;
    }

    double lateralAcceleration_{0.};       // m/s^2
    double longitudinalAcceleration_{0.};  // m/s^2
    double verticalAcceleration{0.};       // m/s^2
};
}  // namespace messages
}  // namespace tcan_can_j1939