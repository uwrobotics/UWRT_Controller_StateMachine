#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// ... [other includes remain the same] ...

/* Custom Message */
#include "uwrt_ros_msg/msg/odrive_cmd.hpp"

using namespace std::chrono_literals;

/**
 * @class StateMachine
 * @brief Manages the lifecycle states of the drivetrain system.
 */
class StateMachine : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit StateMachine(const std::string &node_name, bool intra_process_comms = true)
        : rclcpp_lifecycle::LifecycleNode(
              node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state);

private:
    // Use a lifecycle publisher
    rclcpp_lifecycle::LifecyclePublisher<uwrt_ros_msg::msg::OdriveCmd>::SharedPtr motor_cmd_;

    std::vector<std::string> axis_id_set = {"Left", "Right"};

    bool request_odrive_cmd(const std::string &axis_id, const std::string &cmd, const std::string &payload);
};

#endif // DRIVETRAIN_H
