#include <rclcpp/rclcpp.hpp>

#include <tcan_bridge_msgs/msg/can_frame.hpp>
#include <tcan_can/CanBusManager.hpp>
#include <tcan_can/SocketBus.hpp>
#include <tcan_can/SocketBusOptions.hpp>

#include <tcan_bridge/tcan_bridge.hpp>

namespace tcan_bridge {

class CanToRosBridge : public rclcpp::Node, public TcanBridge {
 public:
  CanToRosBridge() : rclcpp::Node("ros_to_can_bridge"), TcanBridge("can0") {
    canManager_.getCanBus(0)->addCanMessage(tcan_can::CanBus::CanFrameIdentifier(0x0, 0x0), this, &CanToRosBridge::canMsgCallback);
    canManager_.startThreads();

    publisher_ = this->create_publisher<tcan_bridge_msgs::msg::CanFrame>("can0", 10);
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
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tcan_bridge::CanToRosBridge>());
  rclcpp::shutdown();
  return 0;
}