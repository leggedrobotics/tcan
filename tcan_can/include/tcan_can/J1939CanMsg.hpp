#pragma once

#include "CanMsg.hpp"

namespace tcan_can {

/**
 * SAE J1939 Can message
 */
class J1939CanMsg : public CanMsg {
public:

    static const uint32_t CAN_ID_PGN_MASK = 0x3ffff00u;
    static const uint32_t extendedFlag = 1UL << 31;

    constexpr explicit J1939CanMsg(const CanMsg& msg) : CanMsg(msg) {}

    explicit J1939CanMsg(
        uint8_t priority,
        uint32_t pgn,
        uint8_t source_address,
        std::initializer_list<uint8_t> data) 
	    : CanMsg(extendedFlag | static_cast<uint32_t>(priority) << 26 | pgn << 8 | source_address, data) {}

    constexpr uint8_t getPriority() const { return getCobId() >> 26u & 0x7u; }

    constexpr bool getExtendedDataPage() const { return getParameterGroupNumber() >> 17u & 0x1u; }

    constexpr bool getDataPage() const { return getParameterGroupNumber() >> 16u & 0x1u; }

    constexpr uint8_t getPduFormat() const { return getParameterGroupNumber() >> 8u & 0xFFu; }

    constexpr uint8_t getPduSpecific() const { return getParameterGroupNumber() & 0xFFu; }

    constexpr uint8_t getSourceAddress() const { return getCobId() & 0xFFu; }

    constexpr uint32_t getParameterGroupNumber() const {
      return getParameterGroupCanId() >> 8;
    }

    constexpr uint32_t getParameterGroupCanId() const {
      return getCobId() & CAN_ID_PGN_MASK;
    }

    using CanMsg::getCobId;
    using CanMsg::getData;
    using CanMsg::getLength;
};

} // tcan_can
