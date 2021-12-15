#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

#include "tcan_bridge/tcan_bridge.hpp"

namespace tcan_bridge {

class RosToCanBridge : public rclcpp::Node, public TcanBridge {
 public:
  RosToCanBridge() : rclcpp::Node("ros_to_can_bridge"), TcanBridge("can1") {
    subscriber_ = this->create_subscription<tcan_bridge_msgs::msg::CanFrame>(
        "can0", 10, std::bind(&RosToCanBridge::rosMsgCallback, this, std::placeholders::_1));

    canManager_.startThreads();
  }

 private:
  void rosMsgCallback(const tcan_bridge_msgs::msg::CanFrame::SharedPtr msg) {
    tcan_can::CanMsg canMsg(msg->id, msg->length, msg->data.data());
    canManager_.getCanBus(0)->sendMessage(canMsg);
  }

 private:
  rclcpp::Subscription<tcan_bridge_msgs::msg::CanFrame>::SharedPtr subscriber_;
};

}  // namespace tcan_bridge

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::RosToCanBridge>());
  rclcpp::shutdown();
  return 0;
}