#include "../include/drivetrain_state_machine/state_machine.hpp"


bool StateMachine::request_odrive_cmd
(std::string axis_id, std::string cmd, std::string payload) {
  auto request = std::make_shared<uwrt_ros_msg::srv::OdriveCmd::Request>();;
    
  request->axis_id = axis_id;
  request->cmd = cmd;
  request->payload = payload;
  while(motor_cmd_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      std::cout << "return" << std::endl;
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    std::cout << "Failed to connect" << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = motor_cmd_->async_send_request(request);
  std::cout << result.get()->status << std::endl;
  // Verify this is a blocking function and work if there is a server
  return result.get()->status
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_configure(const rclcpp_lifecycle::State &) {
  motor_cmd_ = this->create_client<uwrt_ros_msg::srv::OdriveCmd>("odrive_cmd_service");
  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  bool result = true;
  for(const std::string& axis: axis_id_set) {
    auto result = this->request_odrive_cmd(axis, "Set_Axis_State", "Axis_Requested_State: FULL_CALIBRATION_SEQUENCE;");
    std::cout << axis << std::endl;
  }
  if (result == false) {
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

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_shutdown(const rclcpp_lifecycle::State & state) {
  motor_cmd_.reset();

  RCUTILS_LOG_INFO_NAMED(
    get_name(),
    "on shutdown is called from state %s.",
    state.label().c_str());
    
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<StateMachine> lc_node =
    std::make_shared<StateMachine>("lc_talker");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}