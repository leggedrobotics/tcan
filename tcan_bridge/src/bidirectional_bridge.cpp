#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

#include <tcan_bridge/tcan_bridge.hpp>

namespace tcan_bridge {

class BidirectionalBridge : public rclcpp::Node, public TcanBridge {
 public:
  BidirectionalBridge() = delete;
  BidirectionalBridge(const std::string& nodeName, const std::string& canInterfaceName, const std::string& publishedName,
                      const std::string& subscribedName)
      : rclcpp::Node(nodeName), TcanBridge(canInterfaceName) {
    canManager_.getCanBus(0)->addCanMessage(tcan_can::CanFrameIdentifier(0x0, 0x0), this, &BidirectionalBridge::canMsgCallback);
    canManager_.startThreads();

    publisher_ = this->create_publisher<tcan_bridge_msgs::msg::CanFrame>(publishedName, 10);
    subscribers_.emplace_back(this->create_subscription<tcan_bridge_msgs::msg::CanFrame>(
        subscribedName, 10, std::bind(&BidirectionalBridge::rosMsgCallback, this, std::placeholders::_1)));
  }

  bool canMsgCallback(const tcan_can::CanMsg& cmsg) {
    tcan_bridge_msgs::msg::CanFrame rosMsg;
    rosMsg.id = cmsg.getCobId();
    rosMsg.length = cmsg.getLength();
    rosMsg.data.insert(rosMsg.data.end(), &cmsg.getData()[0], &cmsg.getData()[cmsg.getLength()]);
    publisher_->publish(rosMsg);

    return true;
  }

  void rosMsgCallback(const tcan_bridge_msgs::msg::CanFrame::SharedPtr msg) {
    tcan_can::CanMsg canMsg(msg->id, msg->length, msg->data.data());
    canManager_.getCanBus(0)->sendMessage(canMsg);
  }

 private:
  rclcpp::Publisher<tcan_bridge_msgs::msg::CanFrame>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<tcan_bridge_msgs::msg::CanFrame>::SharedPtr> subscribers_;
};

}  // namespace tcan_bridge

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cout << "Call with\n";
    std::cout << "`ros2 run tcan_bridge bidirectional_bridge <canInterfaceName> <publishedRosTopicName> <subscribedRosTopicName>`\n";
    std::cout << "e.g. `ros2 run tcan_bridge bidirectional_bridge can0 can0AsRos toPublishOnCan0`" << std::endl;
    return -1;
  }
  const std::string canInterfaceName = argv[1];
  const std::string publishedName = argv[2];
  const std::string subscribedName = argv[3];
  const std::string nodeName = canInterfaceName + "_to_" + publishedName + "_and_from_" + subscribedName + "_bridge_";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::BidirectionalBridge>(nodeName, canInterfaceName, publishedName, subscribedName));
  rclcpp::shutdown();
  return 0;
}