#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/cmd_vel.hpp"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
  CmdVelPublisher()
  : Node("cmd_vel_publisher")
  {
    cmd_vel_publisher_ =
      this->create_publisher<tutorial_interfaces::msg::CmdVel>("cmd_vel", 10);

    auto publish_msg = [this]() -> void {
        auto message = tutorial_interfaces::msg::CmdVel();

        message.axis_id = 1;
        message.cmd = "Vel";
        message.speed = 10;

        this->cmd_vel_publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<tutorial_interfaces::msg::CmdVel>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPublisher>());
  rclcpp::shutdown();

  return 0;
}