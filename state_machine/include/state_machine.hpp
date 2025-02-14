#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

/**
 * @file drivetrain.hpp
 * @brief Defines the StateMachine class for managing the lifecycle of the drivetrain system.
 *
 * This class is a ROS 2 LifecycleNode that controls the state transitions of the drivetrain.
 * It includes methods for configuring, activating, deactivating, cleaning up, and shutting down.
 */

/* Standard Library Includes */
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

/* ROS Base Dependencies */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

/* ROS Lifecycle Dependencies */
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

/* ROS Logging Dependencies */
#include "rcutils/logging_macros.h"

/* Custom Message */
#include "uwrt_ros_msg/msg/odrive_cmd.hpp"

using namespace std::chrono_literals;

/**
 * @class StateMachine
 * @brief Manages the lifecycle states of the drivetrain system.
 *
 * This class handles state transitions such as configuration, activation, deactivation,
 * cleanup, and shutdown of the drivetrain system using ROS 2 lifecycle management.
 */
class StateMachine : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Constructor for the StateMachine class.
     * @param node_name Name of the node.
     * @param intra_process_comms Enables intra-process communication if set to true.
     */
    explicit StateMachine(const std::string &node_name, bool intra_process_comms = true)
        : rclcpp_lifecycle::LifecycleNode(
              node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

    /**
     * @brief Handles the configuration state transition.
     *
     * Performs necessary setup steps, including creating a publisher for ODrive commands
     * and sending an initial calibration message for each axis.
     *
     * @param state The current lifecycle state.
     * @return Lifecycle transition success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state);

    /**
     * @brief Handles the activation state transition.
     *
     * Prepares the node for active operation.
     *
     * @param state The current lifecycle state.
     * @return Lifecycle transition success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state);

    /**
     * @brief Handles the deactivation state transition.
     *
     * Pauses the operation of the node.
     *
     * @param state The current lifecycle state.
     * @return Lifecycle transition success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state);

    /**
     * @brief Handles the cleanup state transition.
     *
     * Resets the publisher and cleans up resources.
     *
     * @param state The current lifecycle state.
     * @return Lifecycle transition success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state);

    /**
     * @brief Handles the shutdown state transition.
     *
     * Releases resources and logs the shutdown sequence.
     *
     * @param state The current lifecycle state.
     * @return Lifecycle transition success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state);

private:
    /**
     * @brief ROS publisher for sending ODrive command messages.
     */
    rclcpp::Publisher<uwrt_ros_msg::msg::OdriveCmd>::SharedPtr motor_cmd_;

    /**
     * @brief List of axis identifiers used in the drivetrain system.
     */
    std::vector<std::string> axis_id_set = {"Left", "Right"};

    /**
     * @brief Publishes an ODrive command using a ROS message.
     * @param axis_id The ID of the axis to send the command to.
     * @param cmd The command to be executed.
     * @param payload Additional data for the command.
     * @return True if the message was published successfully, false otherwise.
     */
    bool request_odrive_cmd(const std::string &axis_id, const std::string &cmd, const std::string &payload);
};

#endif // DRIVETRAIN_H
