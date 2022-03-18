#include "tcan_can_j1939/DeviceJ1939.hpp"

#include "tcan_can/CanBus.hpp"
#include "tcan_can_j1939/J1939CanMsg.hpp"

#include <linux/can.h>

namespace tcan_can_j1939 {

bool DeviceJ1939::initDevice() {
    return bus_->addCanMessage(tcan_can::CanFrameIdentifier(CAN_EFF_FLAG | getNodeId(), 0x800000FFU), this, &DeviceJ1939::parseMessage);
}

bool DeviceJ1939::parseMessage(const tcan_can::CanMsg& msg) {
    const auto pgn = static_cast<const J1939CanMsg&>(msg).getParameterGroupNumber();
    std::cout << "Parsing message " << msg.getCobId() << '\n';
    if (pgnMap_.find(pgn) == pgnMap_.end()) {
        std::cout << "Got unknown message\n";
        return true;
    }
    return pgnMap_[pgn]->parse(msg);
}

}  // namespace tcan_can_j1939