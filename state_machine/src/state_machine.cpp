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
    // Create the request
    auto request = std::make_shared<uwrt_ros_msg::srv::OdriveCmd::Request>();
    request->axis_id = axis_id;
    request->cmd = cmd;
    request->payload = payload;

    // Wait for service availability
    while (!motor_cmd_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service. Exiting.");
            return false;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    // Create a promise and get a future to wait for response
    std::promise<int> response_promise;
    std::future<int> response_future = response_promise.get_future();

    // Send the request asynchronously with a callback
    auto future_result = motor_cmd_->async_send_request(request,
        [this, &response_promise](rclcpp::Client<uwrt_ros_msg::srv::OdriveCmd>::SharedFuture future) {
            this->odrive_cmd_response_callback(future, response_promise);
        });

    // Wait for the response in a blocking way, but with a timeout
    if (response_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        return response_future.get();
    } else {
        RCLCPP_ERROR(get_logger(), "Service call timed out.");
        return false; // Return failure if response was not received in time
    }
}

void StateMachine::odrive_cmd_response_callback(
    rclcpp::Client<uwrt_ros_msg::srv::OdriveCmd>::SharedFuture future_result,
    std::promise<int>& response_promise) {

    try {
        auto response = future_result.get();
        RCLCPP_INFO(get_logger(), "Received response: %d", response->status);

        // Fulfill the promise to wake up request_odrive_cmd()
        response_promise.set_value(response->status);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception in callback: %s", e.what());
        response_promise.set_value(0); // Indicate failure
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
    rclcpp::executors::MultiThreadedExecutor exe;
    auto lc_node = std::make_shared<StateMachine>("lc_talker");
    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
