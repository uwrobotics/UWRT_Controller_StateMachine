#include "state_machine.hpp"
#include "rcutils/logging_macros.h"

void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {
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
    
    if(data.contains("Payload")) {
      if(data["Payload"] == "Success") {
        cali_complete = true;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
  } catch (nlohmann::json::parse_error& ex) {
    // Output exception information if parsing fails
    std::cerr << "JSON Parse error: " << ex.what() << std::endl;
    return "";
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
  std::string payload = "";
  std_msgs::msg::String msg;
  msg.data = json_wrapper("Init", "Request", payload);
  if (json_publisher_) {
    json_publisher_->on_activate();
  }
  while(cali_complete == false) {
    RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
    std::string payload = "{"Stage": "Calibration", "Type": "request", "Target": "Drivetrain", "Command": "Set_Axis_State", "Payload": {"1": "FULL_CALIBRATION_SEQUENCE", "2": "FULL_CALIBRATION_SEQUENCE", "3": "FULL_CALIBRATION_SEQUENCE", "4": "FULL_CALIBRATION_SEQUENCE", "5": "FULL_CALIBRATION_SEQUENCE", "6": "FULL_CALIBRATION_SEQUENCE"}}"
    std_msgs::msg::String msg;
    msg.data = payload;
    json_publisher_->publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  cali_complete = false;
  while(cali_complete == false) {
    RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
    std::string payload = "{"Stage": "Calibration", "Type": "request", "Target": "Drivetrain", "Command": "Set_Axis_State", "Payload": {"1": "CLOSED_LOOP_CONTROL", "2": "CLOSED_LOOP_CONTROL", "3": "CLOSED_LOOP_CONTROL", "4": "CLOSED_LOOP_CONTROL", "5": "CLOSED_LOOP_CONTROL", "6": "CLOSED_LOOP_CONTROL"}}"
    std_msgs::msg::String msg;
    msg.data = payload;
    json_publisher_->publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  cali_complete = false;
  while(cali_complete == false) {
    RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
    std::string payload = "{"Stage": "Calibration", "Type": "request", "Target": "Drivetrain", "Command": "Set_Controller_Mode", "Payload": {"1": {"control_mode": "VELOCITY_CONTROL", "input_mode": "VEL_RAMP"}, "2": {"control_mode": "VELOCITY_CONTROL", "input_mode": "VEL_RAMP"}, "3": {"control_mode": "VELOCITY_CONTROL", "input_mode": "VEL_RAMP"}, "4": {"control_mode": "VELOCITY_CONTROL", "input_mode": "VEL_RAMP"}, "5": {"control_mode": "VELOCITY_CONTROL", "input_mode": "VEL_RAMP"}, "6": {"control_mode": "VELOCITY_CONTROL", "input_mode": "VEL_RAMP"}}}"
    std_msgs::msg::String msg;
    msg.data = payload;
    json_publisher_->publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  // implement a mutex msg box 
  if (json_publisher_) {
    json_publisher_->on_deactivate();
  }
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&JointStateToTwistConverter::joint_state_callback, this, std::placeholders::_1));
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
