#include <rclcpp/rclcpp.hpp>

#include <message_logger/message_logger.hpp>
#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

namespace tcan_bridge {

class BidirectionalBridge : public rclcpp::Node {
 public:
  BidirectionalBridge() = delete;
  BidirectionalBridge(std::string& canInterfaceName, std::string& publishedRosTopicName, std::string& subscribedRosTopicName)
      : rclcpp::Node("can_bridge") {
    declare_parameter("can_interface_name");
    get_parameter("can_interface_name", canInterfaceName);
    declare_parameter("published_ros_topic_name");
    get_parameter("published_ros_topic_name", publishedRosTopicName);
    declare_parameter("subscribed_ros_topic_name");
    get_parameter("subscribed_ros_topic_name", subscribedRosTopicName);

    if (canInterfaceName.empty()) {
      MELO_ERROR_STREAM("No CAN interface defined. Exiting");
      exit(1);
    }

    auto options = std::make_unique<tcan_can::SocketBusOptions>(canInterfaceName);
    options->loopback_ = true;
    canManager_.addBus(new tcan_can::SocketBus(std::move(options)));
    canManager_.getCanBus(0)->addCanMessage(tcan_can::CanFrameIdentifier(0x0, 0x0), this, &BidirectionalBridge::canMsgCallback);
    canManager_.startThreads();

    if (!publishedRosTopicName.empty()) {
      MELO_INFO_STREAM("Publishing CAN messages of " << canInterfaceName << " on topic " << publishedRosTopicName);
      publisher_ = this->create_publisher<tcan_bridge_msgs::msg::CanFrame>(publishedRosTopicName, 10);
    } else {
      MELO_INFO_STREAM("Not publishing CAN messages via ROS");
    }
    if (!subscribedRosTopicName.empty()) {
      MELO_INFO_STREAM("Subscribing to ROS messages on topic " << subscribedRosTopicName << " and forwarding to bus "
                                                                << canInterfaceName);
      subscribers_ = this->create_subscription<tcan_bridge_msgs::msg::CanFrame>(
          subscribedRosTopicName, 10, std::bind(&BidirectionalBridge::rosMsgCallback, this, std::placeholders::_1));
    } else {
      MELO_INFO_STREAM("Not forwarding any ROS messages to CAN");
    }
  }

  bool canMsgCallback(const tcan_can::CanMsg& cmsg) {
    if (publisher_ == nullptr || publisher_->get_subscription_count() == 0) {
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
  MELO_INFO_STREAM("tcan_bridge");
  MELO_INFO_STREAM("Call with");
  MELO_INFO_STREAM("`ros2 run tcan_bridge bidirectional_bridge <canInterfaceName> <publishedRosTopicName> <subscribedRosTopicName>`");
  MELO_INFO_STREAM("e.g. `ros2 run tcan_bridge bidirectional_bridge can0 can0AsRos toPublishOnCan0`");
  MELO_INFO_STREAM("Or set ROS parameters 'can_interface_name', 'published_ros_topic_name', subscribed_ros_topic_name'");

  std::string canInterfaceName;
  if (argc >= 2) {
    canInterfaceName = argv[1];
  }
  std::string publishedRosTopicName;
  if (argc >= 3) {
    publishedRosTopicName = argv[2];
  }
  std::string subscribedRosTopicName;
  if (argc >= 4) {
    subscribedRosTopicName = argv[3];
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::BidirectionalBridge>(canInterfaceName, publishedRosTopicName, subscribedRosTopicName));
  rclcpp::shutdown();
}