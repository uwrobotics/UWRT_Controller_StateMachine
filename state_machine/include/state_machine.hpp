#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_

/**
 * @file state_machine.hpp
 * @brief Defines the StateMachine class for managing the lifecycle of the drivetrain system.
 *
 * This class is a ROS 2 LifecycleNode that controls the state transitions of the drivetrain.
 * It includes methods for configuring, activating, deactivating, cleaning up, and shutting down.
 */

/* Standard Library Includes */
#include <memory>
#include <string>
#include <vector>

/* ROS 2 Base Dependencies */
#include "rclcpp/rclcpp.hpp"

/* ROS 2 Lifecycle Dependencies */
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

/* Custom Message */
#include <nlohmann/json.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"


/**
 * Todo:
 * - add description in the message definition
 * - if description == None
 *  - odrive send available interface/ None
 *  - permit to enter activate if only if available interface
 */

/**
 * @class StateMachine
 * @brief Manages the lifecycle states of the drivetrain system.
 */
class StateMachine : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the StateMachine class.
   * @param node_name Name of the node.
   * @param intra_process_comms Enables intra-process communication if set to true.
   */
  explicit StateMachine(const std::string & node_name, bool intra_process_comms = true)
  : rclcpp_lifecycle::LifecycleNode(
        node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
  }

  // Lifecycle callbacks.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// Lifecycle publisher for OdriveCmd messages.
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr json_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  rclcpp::TimerBase::SharedPtr calibration_timer_;

  std::string json_wrapper(const std::string stage, const std::string type, const std::string payload) {
    nlohmann::json msg;
    msg["stage"] = stage;
    msg["type"] = type;
    msg["payload"] = payload;
  
    return msg.dump();
  };

  std::string json_request_wrapper(const std::string stage, const std::string type, const std::string target, const std::string command, const std::string payload_msg) {
    nlohmann::json j;
    j["Stage"] = stage;
    j["Type"] = type;
    j["Target"] = target;
    j["Command"] = command;
    
    // Create the Payload object with keys "1" to "6"
    nlohmann::json payload;
    for (int i = 1; i <= 6; ++i) {
      payload[std::to_string(i)] = payload_msg;
    }
    
    j["Payload"] = payload;
    
    // Convert the JSON object to a string (minified version)
    return j.dump();
  };

  bool cali_complete = false;

  void odrive_json_callback(const std_msgs::msg::String& msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const;
};

#endif  // STATE_MACHINE_HPP_
