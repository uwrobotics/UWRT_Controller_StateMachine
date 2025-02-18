#include "state_machine.hpp"
#include "rcutils/logging_macros.h"

// Publish an Odrive command message.
bool StateMachine::request_odrive_cmd(const std::string & description,
                                      const std::string & axis_id,
                                      const std::string & cmd,
                                      const std::string & payload) {
  auto msg = std::make_shared<uwrt_ros_msg::msg::OdriveCmd>();
  msg->description = description;
  msg->axis_id = axis_id;
  msg->cmd = cmd;
  msg->payload = payload;

  // Publish the message (this will only send messages if the publisher is activated).
  motor_cmd_->publish(*msg);
  RCLCPP_INFO(get_logger(), "Published OdriveCmd: description=%s axis_id='%s', cmd='%s', payload='%s'",
              description.c_str(), axis_id.c_str(), cmd.c_str(), payload.c_str());
  return true;
}

bool StateMachine::response_callback(const uwrt_ros_msg::msg::MsgResponse & msg) const {
  RCLCPP_INFO(this->get_logger(), "Msg Response: %d timestamp: %s", msg.status, msg.timestamp.c_str());
  return msg.status;
}

std::string StateMachine::odrive_json_callback(const std_msgs::msg::String& msg) const {
  RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
  return msg.data;
}

void StateMachine::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {
  RCLCPP_INFO(this->get_logger(), "Received joint state message with %zu joints", msg->name.size());
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
  if (motor_cmd_) {
    motor_cmd_->on_activate();
    json_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Msg Response: %s", msg.data.c_str());
  json_publisher_->publish(msg);
  // implement a mutex msg box 
  if (motor_cmd_) {
    motor_cmd_->on_deactivate();
    json_publisher_->on_deactivate();
  }
  motor_cmd_ = this->create_publisher<uwrt_ros_msg::msg::OdriveCmd>("OdriveCmd", 10);
  cmd_response_ = this->create_subscription<uwrt_ros_msg::msg::MsgResponse>(
    "MsgResponse", 10, std::bind(&StateMachine::response_callback, this, std::placeholders::_1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10, std::bind(&StateMachine::joint_state_callback, this, std::placeholders::_1));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_activate: Activate the lifecycle publisher so that messages are forwarded.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  if (motor_cmd_) {
    motor_cmd_->on_activate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_deactivate: Deactivate the publisher.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  if (motor_cmd_) {
    motor_cmd_->on_deactivate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_cleanup: Clean up the publisher.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_cleanup(const rclcpp_lifecycle::State &) {
  motor_cmd_.reset();
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// on_shutdown: Shut down the node.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateMachine::on_shutdown(const rclcpp_lifecycle::State & state) {
  motor_cmd_.reset();
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
