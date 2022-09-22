
#include <ros/ros.h>

#include <tcan_bridge_msgs_ros1/CanFrame.h>



#include <message_logger/message_logger.hpp>

#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

namespace tcan_bridge {

/*
  setupCan()
  setupRos()

  typedef for RosMsg

  different type for publisher and subscriber
*/

class BidirectionalBridge {
 public:
  BidirectionalBridge(int) {
    ros::NodeHandle nh;

    std::string canInterfaceName;
    std::string publishedRosTopicName;
    std::string subscribedRosTopicName;

    ros::param::get(ros::this_node::getName() + "/can_interface_name", canInterfaceName);
    ros::param::get(ros::this_node::getName() + "/published_ros_topic_name", publishedRosTopicName);
    ros::param::get(ros::this_node::getName() + "/subscribed_ros_topic_name", subscribedRosTopicName);

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
      publisher_ = new ros::Publisher(nh.advertise<tcan_bridge_msgs_ros1::CanFrame>(publishedRosTopicName, 1000));
    } else {
      MELO_INFO_STREAM("Not publishing CAN messages via ROS");
    }
    if (!subscribedRosTopicName.empty()) {
      MELO_INFO_STREAM("Subscribing to ROS messages on topic " << subscribedRosTopicName << " and forwarding to bus " << canInterfaceName);
      subscribers_ = new ros::Subscriber(nh.subscribe(subscribedRosTopicName, 1000, &BidirectionalBridge::rosMsgCallback, this));
    } else {
      MELO_INFO_STREAM("Not forwarding any ROS messages to CAN");
    }
  }

  bool canMsgCallback(const tcan_can::CanMsg& cmsg) {
    std::lock_guard<std::mutex> guard(mutexCan_);
    if (publisher_ == nullptr || publisher_->getNumSubscribers() == 0) {
      return true;
    }
    tcan_bridge_msgs_ros1::CanFrame rosMsg;
    rosMsg.id = cmsg.getCobId();
    rosMsg.length = cmsg.getLength();
    rosMsg.data.insert(rosMsg.data.end(), &cmsg.getData()[0], &cmsg.getData()[cmsg.getLength()]);
    publisher_->publish(rosMsg);

    return true;
  }

  void rosMsgCallback(const tcan_bridge_msgs_ros1::CanFrame::ConstPtr msg) {
    std::lock_guard<std::mutex> guard(mutexRos_);
    tcan_can::CanMsg canMsg(msg->id, msg->length, msg->data.data());
    canManager_.getCanBus(0)->sendMessage(canMsg);
  }

 private:
  ros::Publisher* publisher_{nullptr};
  ros::Subscriber* subscribers_{nullptr};

  std::mutex mutexCan_;
  std::mutex mutexRos_;

  tcan_can::CanBusManager canManager_;
};

/*
class BidirectionalBridgeRos2 : rclcpp::Node {
  public:
   BidirectionalBridgeRos2() = delete;
   BidirectionalBridgeRos2(std::string & canInterfaceName, std::string & publishedRosTopicName, std::string & subscribedRosTopicName)
       : rclcpp::Node("can_interface_name") {
     declare_parameter("can_interface_name");
     get_parameter("can_interface_name", canInterfaceName);
     declare_parameter("published_ros_topic_name");
     get_parameter("published_ros_topic_name", publishedRosTopicName);
     declare_parameter("subscribed_ros_topic_name");
     get_parameter("subscribed_ros_topic_name", subscribedRosTopicName);
  }
};
*/

}  // namespace tcan_bridge

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tcan_bridge");
  std::cout << "a\n";
  tcan_bridge::BidirectionalBridge bridge(0);
  ros::spin();
}