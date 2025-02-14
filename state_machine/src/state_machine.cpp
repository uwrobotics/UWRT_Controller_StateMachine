#include "state_machine.hpp"

bool StateMachine::request_odrive_cmd(const std::string &axis_id, const std::string &cmd, const std::string &payload) {
    auto msg = std::make_shared<uwrt_ros_msg::msg::OdriveCmd>();
    msg->axis_id = axis_id;
    msg->cmd = cmd;
    msg->payload = payload;

    // Publish the message.
    motor_cmd_->publish(*msg);
    RCLCPP_INFO(get_logger(), "Published OdriveCmd: axis_id='%s', cmd='%s', payload='%s'", 
                axis_id.c_str(), cmd.c_str(), payload.c_str());

    return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_configure(const rclcpp_lifecycle::State &) {
    // Create a lifecycle publisher with QoS depth of 10.
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_activate(const rclcpp_lifecycle::State &) {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    // Activate the lifecycle publisher so that it will actually publish messages.
    if (motor_cmd_) {
        motor_cmd_->on_activate();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_deactivate(const rclcpp_lifecycle::State &) {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    if (motor_cmd_) {
        motor_cmd_->on_deactivate();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_cleanup(const rclcpp_lifecycle::State &) {
    motor_cmd_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_shutdown(const rclcpp_lifecycle::State &state) {
    motor_cmd_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

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
