#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

#include <tcan_bridge/tcan_bridge.hpp>

namespace tcan_bridge {

class CanToRosBridge : public rclcpp::Node, public TcanBridge {
 public:
  CanToRosBridge() = delete;
  CanToRosBridge(const std::string& nodeName, const std::string& canInterfaceName, const std::string& rosTopicName)
      : rclcpp::Node(nodeName), TcanBridge(canInterfaceName) {
    canManager_.getCanBus(0)->addCanMessage(tcan_can::CanFrameIdentifier(0x0, 0x0), this, &CanToRosBridge::canMsgCallback);
    canManager_.startThreads();

    publisher_ = this->create_publisher<tcan_bridge_msgs::msg::CanFrame>(rosTopicName, 10);
  }

  bool canMsgCallback(const tcan_can::CanMsg& cmsg) {
    tcan_bridge_msgs::msg::CanFrame rosMsg;
    rosMsg.id = cmsg.getCobId();
    rosMsg.length = cmsg.getLength();
    rosMsg.data.insert(rosMsg.data.end(), &cmsg.getData()[0], &cmsg.getData()[cmsg.getLength()]);
    publisher_->publish(rosMsg);

    return true;
  }

 private:
  rclcpp::Publisher<tcan_bridge_msgs::msg::CanFrame>::SharedPtr publisher_;
};

}  // namespace tcan_bridge

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cout << "Call with\n";
    std::cout << "`ros2 run tcan_bridge can_to_ros_bridge <canInterfaceName> <rostopicName>`\n";
    std::cout << "e.g. `ros2 run tcan_bridge can_to_ros_bridge can0 can0AsRos`" << std::endl;
    return -1;
  }
  const std::string canInterfaceName = argv[1];
  const std::string rosTopicName = argv[2];
  const std::string nodeName = "can_" + canInterfaceName + "_to_rostopic_" + rosTopicName + "_bridge_";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::CanToRosBridge>(nodeName, canInterfaceName, rosTopicName));
  rclcpp::shutdown();
  return 0;
}