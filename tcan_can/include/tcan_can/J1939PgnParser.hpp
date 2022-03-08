#pragma once

#include <cstdint>

#include "tcan_can/CanMsg.hpp"

namespace tcan_can {
struct J1939PgnParser {
    J1939PgnParser() = delete;
    J1939PgnParser(uint32_t pgn) : pgn_(pgn) {}
    virtual ~J1939PgnParser() = default;

    static double scaledMessageFromRaw(double raw, double resolution, double offset) {
        std::cout << raw << " * " << resolution << " + " << offset << '\n';
        return raw * resolution + offset;
    }

    const uint32_t pgn_;
    virtual bool parse(const tcan_can::CanMsg& msg) = 0;
};
}  // namespace tcan_can