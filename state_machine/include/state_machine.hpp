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
#include "uwrt_ros_msg/msg/odrive_cmd.hpp"
#include "uwrt_ros_msg/msg/msg_response.hpp"

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
  rclcpp_lifecycle::LifecyclePublisher<uwrt_ros_msg::msg::OdriveCmd>::SharedPtr motor_cmd_;

  rclcpp::Subscription<uwrt_ros_msg::msg::MsgResponse>::SharedPtr cmd_response_;

  /// List of axis identifiers used in the drivetrain system.
  std::vector<std::string> axis_id_set_ = {"Left", "Right"};

  /**
   * @brief Publishes an Odrive command using a ROS message.
   * @param axis_id The ID of the axis.
   * @param cmd The command.
   * @param payload Additional command data.
   * @return True if the message was published successfully.
   */
  bool request_odrive_cmd(const std::string & axis_id,
                          const std::string & cmd,
                          const std::string & payload);

  bool response_callback(const uwrt_ros_msg::msg::MsgResponse & msg) const;
};

#endif  // STATE_MACHINE_HPP_
