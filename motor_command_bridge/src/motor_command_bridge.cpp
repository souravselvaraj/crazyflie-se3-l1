#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>   // <- correct include for gz-msgs10 / Harmonic

class MotorCommandBridge : public rclcpp::Node
{
public:
  MotorCommandBridge()
  : Node("motor_command_bridge")
  {
    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/motor_speed",
      10,
      std::bind(&MotorCommandBridge::motorCallback, this, std::placeholders::_1));

    pub_ = node_.Advertise<gz::msgs::Actuators>("/crazyflie/gazebo/command/motor_speed");
    RCLCPP_INFO(this->get_logger(),
                "MotorCommandBridge ready → Publishing to Gazebo topic /crazyflie/gazebo/command/motor_speed");
  }

private:
  void motorCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4)
    {
      RCLCPP_WARN(this->get_logger(), "Expected 4 motor speeds, got %zu", msg->data.size());
      return;
    }

    gz::msgs::Actuators act;
    act.mutable_velocity()->Resize(4, 0.0);
    for (size_t i = 0; i < 4; ++i)
      act.set_velocity(i, msg->data[i]);

    pub_.Publish(act);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorCommandBridge>());
  rclcpp::shutdown();
  return 0;
}
