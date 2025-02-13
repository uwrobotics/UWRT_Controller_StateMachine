#include "../include/state_machine.hpp"

bool StateMachine::request_odrive_cmd(std::string axis_id, std::string cmd, std::string payload) {
  // Create a request object
  auto request = std::make_shared<uwrt_ros_msg::srv::OdriveCmd::Request>();
  request->axis_id = axis_id;
  request->cmd = cmd;
  request->payload = payload;

  // Wait for service to be available
  while (!motor_cmd_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return false;  // Service is not available, return false
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  // Send the asynchronous request
  auto result = motor_cmd_->async_send_request(request);
  // Wait for the result to be ready
  result.wait_for(2s);

  if (result.valid() && result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      // If the result is ready, handle it
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response: %d", result.get()->status);
      return result.get()->status;  // Adjust this as needed based on expected status
  } else {
      std::cout << "No response received in time" << std::endl;
      // If the result did not arrive in time, return false
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Request timed out");
      return false;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_configure(const rclcpp_lifecycle::State &) {
  motor_cmd_ = this->create_client<uwrt_ros_msg::srv::OdriveCmd>("OdriveCmd");
  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  bool success = true; // Initialize the success flag to true
  for (const std::string& axis : axis_id_set) {
      bool result = this->request_odrive_cmd(axis, "Set_Axis_State", "Axis_Requested_State: FULL_CALIBRATION_SEQUENCE;");
      std::cout << "Sending request for axis: " << axis << std::endl;

      // If any request fails, set success to false
      if (!result) {
          RCLCPP_ERROR(get_logger(), "Request for axis %s failed", axis.c_str());
          success = false;
      }
  }

  if (!success) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_activate(const rclcpp_lifecycle::State &) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_deactivate(const rclcpp_lifecycle::State &) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_cleanup(const rclcpp_lifecycle::State &) {
  motor_cmd_.reset();
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_shutdown(const rclcpp_lifecycle::State &state) {
  motor_cmd_.reset();
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown is called from state %s.", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<StateMachine> lc_node = std::make_shared<StateMachine>("lc_talker");
  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
