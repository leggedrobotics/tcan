#pragma once

#include <tcan_can/J1939PgnParser.hpp>

namespace tcan_can_j1939_devices {
struct AngularRateInformation : public tcan_can::J1939PgnParser {
    AngularRateInformation() : tcan_can::J1939PgnParser(0xF02A) {}

    bool parse(const tcan_can::CanMsg& msg) {
        pitchRate_ = scaledMessageFromRaw(msg.readuint16(0), 1. / 128., -250.);
        rollRate_ = scaledMessageFromRaw(msg.readuint16(2), 1. / 128., -250.);
        yawRate_ = scaledMessageFromRaw(msg.readuint16(4), 1 / 128., -250.);
        return true;
    }

    double pitchRate_{0.};
    double rollRate_{0.};
    double yawRate_{0.};
};
}  // namespace tcan_can_j1939_devices