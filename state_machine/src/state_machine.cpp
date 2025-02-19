#include "state_machine.hpp"
#include "rcutils/logging_macros.h"

void StateMachine::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {
  if (msg->velocity.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received JointState with less than two velocity values.");
    return;
  }

  // Assume first velocity is left motor, second is right motor.
  double left_speed  = msg->velocity[0];
  double right_speed = msg->velocity[1];
}

void StateMachine::odrive_json_callback(const std_msgs::msg::String& msg){
  try {
    // Parse the JSON string into a json object
    nlohmann::json data = nlohmann::json::parse(msg.data);
    std::cout << msg.data << std::endl;
    if(data.contains("Payload")) {
      if(data["Payload"] == "Success") {
        cali_complete = true;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
  } catch (nlohmann::json::parse_error& ex) {
    // Output exception information if parsing fails
    std::cerr << "JSON Parse error: " << ex.what() << std::endl;
  }
}

// on_configure: Create the lifecycle publisher and send an initial message.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_configure(const rclcpp_lifecycle::State &) {
  // Create a lifecycle publisher with a QoS depth of 10.
  json_publisher_ = this->create_publisher<std_msgs::msg::String>("OdriveJsonSub", 10);
  json_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "OdriveJsonPub", 10, std::bind(&StateMachine::odrive_json_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  if (json_publisher_) {
    json_publisher_->on_activate();
  }
  // You can alternate between two types of messages or use separate timers.
  std::string payload = json_request_wrapper("Calibration", "request", "Drivetrain", "Set_Axis_State", "FULL_CALIBRATION_SEQUENCE");
  std_msgs::msg::String msg;
  msg.data = payload;
  RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
  json_publisher_->publish(msg);
  while (!cali_complete) {}
  cali_complete = false;
  std::string payload = json_request_wrapper("Calibration", "request", "Drivetrain", "Set_Axis_State", "CLOSE_LOOP_CONTROL");
  std_msgs::msg::String msg;
  msg.data = payload;
  RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
  json_publisher_->publish(msg);
  while (!cali_complete) {}
  // implement a mutex msg box 
  if (json_publisher_) {
    json_publisher_->on_deactivate();
  }
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&StateMachine::joint_state_callback, this, std::placeholders::_1));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_activate: Activate the lifecycle publisher so that messages are forwarded.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  if (json_publisher_) {
    json_publisher_->on_activate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_deactivate: Deactivate the publisher.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  if (json_publisher_) {
    json_publisher_->on_deactivate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_cleanup: Clean up the publisher.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_shutdown: Shut down the node.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_shutdown(const rclcpp_lifecycle::State & state) {
  // motor_cmd_.reset();
  RCLCPP_INFO(get_logger(), "on_shutdown() is called from state %s.", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Main entry point.
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  // Create a MultiThreadedExecutor to allow concurrent callbacks.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto lc_node = std::make_shared<StateMachine>("lc_talker");
  executor.add_node(lc_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
