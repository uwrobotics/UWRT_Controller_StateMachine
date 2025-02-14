#include "state_machine.hpp"

/**
 * @file state_machine.cpp
 * @brief Implements the StateMachine class for managing the drivetrain lifecycle.
 */

/**
 * @brief Sends an ODrive command via a ROS service request.
 * 
 * This method constructs a request message and sends it asynchronously
 * to control an ODrive motor. It ensures that the service is available before
 * sending the request and waits for a response.
 * 
 * @param axis_id The identifier of the motor axis.
 * @param cmd The command to execute on the motor.
 * @param payload Additional command parameters.
 * @return True if the request was successful, false otherwise.
 */
bool StateMachine::request_odrive_cmd(const std::string &axis_id, const std::string &cmd, const std::string &payload) {
    auto request = std::make_shared<uwrt_ros_msg::srv::OdriveCmd::Request>();

    request->axis_id = axis_id;
    request->cmd = cmd;
    request->payload = payload;

    while (!motor_cmd_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto result = motor_cmd_->async_send_request(request);
    auto future = result.wait_for(2s);

    // if (result.valid() && result.wait_for(2s) == std::future_status::ready) {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response: %d", result.get()->status);
    //     return result.get()->status;
    // } else {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Request timed out");
    //     std::cout << result.valid()->status << std::endl;
    //     return false;
    // }
    if (future == std::future_status::ready) {
        auto response = result.get();
        RCLCPP_INFO(this->get_logger(), "Service Response: %d", response->status);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
}

/**
 * @brief Configures the state machine and initializes motor communication.
 * 
 * This method initializes the service client for motor commands and sends a
 * calibration request for each axis.
 * 
 * @return SUCCESS if configuration is successful, FAILURE otherwise.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_configure(const rclcpp_lifecycle::State &) {
    motor_cmd_ = this->create_client<uwrt_ros_msg::srv::OdriveCmd>("OdriveCmd");
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    bool success = true;
    for (const auto &axis : axis_id_set) {
        if (!request_odrive_cmd(axis, "Set_Axis_State", "Axis_Requested_State: FULL_CALIBRATION_SEQUENCE;")) {
            RCLCPP_ERROR(get_logger(), "Request for axis %s failed", axis.c_str());
            success = false;
        }
    }
    return success ? rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS : 
                     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}

/**
 * @brief Activates the state machine, making it operational.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_activate(const rclcpp_lifecycle::State &) {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief Deactivates the state machine, pausing its operation.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_deactivate(const rclcpp_lifecycle::State &) {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief Cleans up resources and resets the service client.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_cleanup(const rclcpp_lifecycle::State &) {
    motor_cmd_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup is called.");
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
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown is called from state %s.", state.label().c_str());
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
    rclcpp::executors::SingleThreadedExecutor exe;
    auto lc_node = std::make_shared<StateMachine>("lc_talker");
    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
