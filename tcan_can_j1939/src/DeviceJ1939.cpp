#include "tcan_can_j1939/DeviceJ1939.hpp"

#include "tcan_can/CanBus.hpp"
#include "tcan_can_j1939/J1939CanMsg.hpp"

namespace tcan_can_j1939 {

bool DeviceJ1939::initDevice() {
    return bus_->addCanMessage(tcan_can::CanFrameIdentifier(getNodeId(), 0xFF), this, &DeviceJ1939::parseMessage);
}

bool DeviceJ1939::parseMessage(const tcan_can::CanMsg& msg) {
    const auto pgn = static_cast<const J1939CanMsg&>(msg).getParameterGroupNumber();
    std::cout << "Parsing message " << msg.getCobId() << '\n';
    return pgnMap_[pgn]->parse(msg);
}

}  // namespace tcan_can_j1939