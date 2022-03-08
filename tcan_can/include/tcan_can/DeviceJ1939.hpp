#pragma once

#include <tcan_can/CanBus.hpp>
#include <tcan_can/CanDevice.hpp>
#include <tcan_can/CanFrameIdentifier.hpp>

namespace tcan_can {

namespace utils {
double scaledMessageFromRaw(double raw, double resolution, double offset) {
    return raw / resolution + offset;
}
}  // namespace utils

class DeviceJ1939 : public tcan_can::CanDevice {
   public:
    static const uint32_t CAN_ID_PGN_MASK = 0x3ffff00u;
    static const uint32_t extendedFlag = 1UL << 31;

    using tcan_can::CanDevice::CanDevice;
    ~DeviceJ1939() override = default;

    template <class T>
    bool addPgn(uint32_t pgn, T* device, bool (std::common_type<T>::type::*fp)(const CanMsg&)) {
        const uint32_t cobId = extendedFlag | pgn << 8 | getNodeId();
        const uint32_t mask = 0xD3FFFFFFu;  // Ignore priority (bits 29-27)
        return bus_->addCanMessage(CanFrameIdentifier(cobId, mask), device, fp);
    }
};

}  // namespace tcan_can