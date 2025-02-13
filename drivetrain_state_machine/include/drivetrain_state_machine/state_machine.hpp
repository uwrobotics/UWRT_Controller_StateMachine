#ifndef DRIVETRAIN
#define DRIVETRAIN

/** 
on_configure: 
- request bus status (.srv)
- send calibration request (.srv) [assert state transition]
    - Full Axis calibration
    - Direction calibration
    - Encoder offsite Calibration
- on_activate:
    - state machine main frame
        - assert current control mode
            - send out command request
    - accept any odrive command?
- on_deactivate:
    - Set mode to idle
    - Assert if error
        - clear_error
        - roll back to on_configure
- clean_up
    - request bus off
*/

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rcutils/logging_macros.h"

/** import Odrive Service Msg **/
#include "uwrt_ros_msg/srv/odrive_cmd.hpp"

using namespace std::chrono_literals;

class StateMachine : public rclcpp_lifecycle::LifecycleNode {
    public:
        /**
         * The StateMachine/lifecyclenode constructor has the same
         * arguments a regular node.
         */
        explicit StateMachine(const std::string & node_name, bool intra_process_comms = false)
        :rclcpp_lifecycle::LifecycleNode(node_name,
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)){
        }


        bool StateMachine::request_odrive_cmd(std::string axis_id, std::string cmd, std::string payload);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state);

    private:
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<uwrt_ros_msg::srv::OdriveCmd>> motor_cmd_;

        std::vector<std::string> axis_id_set = {"Left", "Right"};
};


#endif /**DRIVETRAIN */