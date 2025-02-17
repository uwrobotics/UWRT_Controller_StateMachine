# UWRT_Controller_StateMachine

## Note:

Axis to Joint should support list so we can map left side motor to axis 1,2,3

and right side motor to axis 4,5,6

so Ros2 control (axis_id == Leftside | axis_id == Rightside)

This node keep the same it's just impl lifecycle as statemachine

The odrive responsible for final conversion

```bash
sudo apt-get install nlohmann-json3-dev

source ros_entrypoint.sh
source ./install/setup.bash
ros2 run py_odrive msg_server
ros2 run state_machine state_machine_talker

ros2 lifecycle set /lc_talker configure
ros2 lifecycle set /lc_talker activate
```

## Dependencies

```bash
pip install python-can pyserial cantools pyyaml
```
