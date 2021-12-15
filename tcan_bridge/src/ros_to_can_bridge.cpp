#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

#include "tcan_bridge/tcan_bridge.hpp"

namespace tcan_bridge {

class RosToCanBridge : public rclcpp::Node, public TcanBridge {
 public:
  RosToCanBridge() = delete;
  RosToCanBridge(const std::string& nodeName, const std::string& rosTopicName, const std::string& canInterfaceName)
      : rclcpp::Node(nodeName), TcanBridge(canInterfaceName) {
    subscribers_.emplace_back(this->create_subscription<tcan_bridge_msgs::msg::CanFrame>(
        rosTopicName, 10, std::bind(&RosToCanBridge::rosMsgCallback, this, std::placeholders::_1)));

    canManager_.startThreads();
  }

 private:
  void rosMsgCallback(const tcan_bridge_msgs::msg::CanFrame::SharedPtr msg) {
    tcan_can::CanMsg canMsg(msg->id, msg->length, msg->data.data());
    canManager_.getCanBus(0)->sendMessage(canMsg);
  }

 private:
  std::vector<rclcpp::Subscription<tcan_bridge_msgs::msg::CanFrame>::SharedPtr> subscribers_;
};

}  // namespace tcan_bridge

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cout << "Call with\n";
    std::cout << "`ros2 run tcan_bridge ros_to_can_bridge <rostopicName> <canInterfaceName>`\n";
    std::cout << "e.g. `ros2 run tcan_bridge ros_to_can_bridge can0AsRos can0`" << std::endl;
    return -1;
  }
  const std::string rosTopicName = argv[1];
  const std::string canInterfaceName = argv[2];
  const std::string nodeName = "rostopic_" + rosTopicName + "_to_can_" + canInterfaceName + "_bridge_";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::RosToCanBridge>("ros_to_can_bridge", rosTopicName, canInterfaceName));
  rclcpp::shutdown();
  return 0;
}