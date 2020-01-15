#pragma once

#include "CanMsg.hpp"

namespace tcan_can {

/**
 * SAE J1939 Can message
 */
class SaeCanMsg : public CanMsg {
public:

    static const uint32_t CAN_ID_PGN_MASK = 0x3ffff00u;

    constexpr explicit SaeCanMsg(const CanMsg& msg) : CanMsg(msg) {}

    constexpr uint8_t getPriority() const { return getCobId() >> 26u & 0x7u; }

    constexpr bool getExtendedDataPage() const { return getCobId() >> 25u & 0x1u; }

    constexpr bool getDataPage() const { return getCobId() >> 24u & 0x1u; }

    constexpr uint8_t getPduFormat() const { return getCobId() >> 16u & 0xFFu; }

    constexpr uint8_t getPduSpecific() const { return getCobId() >> 8u & 0xFFu; }

    constexpr uint8_t getSourceAddress() const { return getCobId() & 0xFFu; }

    constexpr uint32_t getParameterGroupNumber() const {
      return getExtendedDataPage() << 17u |
             getDataPage() << 16u |
             getPduFormat() << 8u |
             getPduSpecific();
    }

    constexpr uint32_t getParameterGroupCanId() const {
      return getParameterGroupNumber() << 8u;
    }

    using CanMsg::getCobId;
    using CanMsg::getData;
    using CanMsg::getLength;
};

} // tcan_can