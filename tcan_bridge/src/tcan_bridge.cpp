#include "tcan_bridge/tcan_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

namespace tcan_bridge {

TcanBridge::TcanBridge(std::string interfaceName) {
  auto options = std::make_unique<tcan_can::SocketBusOptions>(interfaceName);
  options->loopback_ = true;
  canManager_.addBus(new tcan_can::SocketBus(std::move(options)));
}

}  // namespace tcan_bridge