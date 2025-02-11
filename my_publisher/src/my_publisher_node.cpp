#include "rclcpp/rclcpp.hpp"
#include "msg/my_custom_msg.hpp"
#include "geometry_msgs/msg/pose.hpp"

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("my_publisher")
    {
        publisher_ = this->create_publisher<my_publisher::msg::MyCustomMsg>("my_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyPublisher::publishMessage, this));
    }

private:
    void publishMessage()
    {
        auto message = my_publisher::msg::MyCustomMsg();
        message.my_integer = 42; // Example value

        // Set example Pose values
        message.my_pose.position.x = 1.0;
        message.my_pose.position.y = 2.0;
        message.my_pose.position.z = 3.0;
        message.my_pose.orientation.x = 0.0;
        message.my_pose.orientation.y = 0.0;
        message.my_pose.orientation.z = 0.0;
        message.my_pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.my_integer);
        publisher_->publish(message);
    }

    rclcpp::Publisher<my_publisher::msg::MyCustomMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}