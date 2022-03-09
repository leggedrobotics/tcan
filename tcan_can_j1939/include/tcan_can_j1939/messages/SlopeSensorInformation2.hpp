#pragma once

#include "tcan_can_j1939/J1939PgnParser.hpp"

namespace tcan_can_j1939 {
namespace messages {
struct SlopeSensorInformation2 : public J1939PgnParser {
    SlopeSensorInformation2() : J1939PgnParser(0xF029) {}

    bool parse(const tcan_can::CanMsg& msg) {
        pitchAngle_ = scaledMessageFromRaw(msg.readuint24(0), 1. / 32768., -250.);
        rollAngle_ = scaledMessageFromRaw(msg.readuint24(3), 1. / 32768., -250.);
        return true;
    }

    double pitchAngle_{0.};
    double rollAngle_{0.};
};
}  // namespace messages
}  // namespace tcan_can_j1939