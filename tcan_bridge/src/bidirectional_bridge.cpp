#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

namespace tcan_bridge {

class BidirectionalBridge : public rclcpp::Node {
 public:
  BidirectionalBridge() = delete;
  BidirectionalBridge(const std::string& nodeName, const std::string& canInterfaceName, const std::string& publishedRosTopicName,
                      const std::string& subscribedRosTopicName) : rclcpp::Node(nodeName) {
    auto options = std::make_unique<tcan_can::SocketBusOptions>(canInterfaceName);
    options->loopback_ = true;
    canManager_.addBus(new tcan_can::SocketBus(std::move(options)));

    canManager_.getCanBus(0)->addCanMessage(tcan_can::CanFrameIdentifier(0x0, 0x0), this, &BidirectionalBridge::canMsgCallback);
    canManager_.startThreads();

    publisher_ = this->create_publisher<tcan_bridge_msgs::msg::CanFrame>(publishedRosTopicName, 10);
    subscribers_ = this->create_subscription<tcan_bridge_msgs::msg::CanFrame>(
        subscribedRosTopicName, 10, std::bind(&BidirectionalBridge::rosMsgCallback, this, std::placeholders::_1));
  }

  bool canMsgCallback(const tcan_can::CanMsg& cmsg) {
    if (publisher_->get_subscription_count() == 0) {
      return true;
    }
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
  rclcpp::Subscription<tcan_bridge_msgs::msg::CanFrame>::SharedPtr subscribers_;

  tcan_can::CanBusManager canManager_;
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
  const std::string publishedRosTopicName = argv[2];
  const std::string subscribedRosTopicName = argv[3];
  const std::string nodeName = canInterfaceName + "_to_" + publishedRosTopicName + "_and_from_" + subscribedRosTopicName + "_bridge_";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::BidirectionalBridge>(nodeName, canInterfaceName, publishedRosTopicName, subscribedRosTopicName));
  rclcpp::shutdown();
  return 0;
}