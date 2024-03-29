#pragma once

#include "tcan_can_j1939/J1939PgnParser.hpp"

namespace tcan_can_j1939 {
namespace messages {
struct AngularRateInformation : public J1939PgnParser {
    AngularRateInformation() : J1939PgnParser(0xF02A) {}

    bool parse(const tcan_can::CanMsg& msg) override {
        pitchRate_ = scaledMessageFromRaw(msg.readuint16(0), 1. / 128., -250.);
        rollRate_ = scaledMessageFromRaw(msg.readuint16(2), 1. / 128., -250.);
        yawRate_ = scaledMessageFromRaw(msg.readuint16(4), 1 / 128., -250.);
        return true;
    }

    double pitchRate_{0.};  // deg/s
    double rollRate_{0.};   // deg/s
    double yawRate_{0.};    // deg/s
};
}  // namespace messages
}  // namespace tcan_can_j1939