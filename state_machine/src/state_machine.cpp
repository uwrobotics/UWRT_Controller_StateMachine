#include "state_machine.hpp"

/**
 * @brief Publishes an ODrive command via a ROS message.
 * 
 * This method constructs a message and publishes it asynchronously
 * to control an ODrive motor.
 * 
 * @param axis_id The identifier of the motor axis.
 * @param cmd The command to execute on the motor.
 * @param payload Additional command parameters.
 * @return True if the message was published successfully, false otherwise.
 */
bool StateMachine::request_odrive_cmd(const std::string &axis_id, const std::string &cmd, const std::string &payload) {
    // Create the message
    auto msg = std::make_shared<uwrt_ros_msg::msg::OdriveCmd>();
    msg->axis_id = axis_id;
    msg->cmd = cmd;
    msg->payload = payload;

    // Publish the message.
    motor_cmd_->publish(*msg);
    RCLCPP_INFO(get_logger(), "Published OdriveCmd: axis_id='%s', cmd='%s', payload='%s'", 
                axis_id.c_str(), cmd.c_str(), payload.c_str());

    // Since publishing is fire-and-forget, return true if the publish() function is called.
    return true;
}

/**
 * @brief Configures the state machine and initializes motor communication.
 * 
 * This method initializes the publisher for motor commands and sends a
 * calibration message for each axis.
 * 
 * @return SUCCESS if configuration is successful, FAILURE otherwise.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_configure(const rclcpp_lifecycle::State &) {
    // Create a publisher for the OdriveCmd message.
    // Using a QoS history depth of 10.
    motor_cmd_ = this->create_publisher<uwrt_ros_msg::msg::OdriveCmd>("OdriveCmd", 10);
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    bool success = true;
    for (const auto &axis : axis_id_set) {
        if (!request_odrive_cmd(axis, "Set_Axis_State", "Axis_Requested_State: FULL_CALIBRATION_SEQUENCE;")) {
            RCLCPP_ERROR(get_logger(), "Request for axis %s failed", axis.c_str());
            success = false;
        }
    }
    return success ? 
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS : 
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}

/**
 * @brief Activates the state machine, making it operational.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_activate(const rclcpp_lifecycle::State &) {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief Deactivates the state machine, pausing its operation.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_deactivate(const rclcpp_lifecycle::State &) {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief Cleans up resources and resets the publisher.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_cleanup(const rclcpp_lifecycle::State &) {
    motor_cmd_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief Handles shutdown sequence, releasing resources and logging state.
 * 
 * @param state The current lifecycle state during shutdown.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_shutdown(const rclcpp_lifecycle::State &state) {
    motor_cmd_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief Main entry point for the state machine node.
 * 
 * Initializes the ROS node and runs the lifecycle state machine.
 */
int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;
    auto lc_node = std::make_shared<StateMachine>("lc_talker");
    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
