# This message presents the current state of a controller of
# a one degree of freedom integrating security and safety supervision.

# Definition: https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/SingleDOFState.msg
#control_msgs/SingleDOFState dof_state #Not yet used in humble
# Definition: https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/msg/JointControllerState.msg
control_msgs/JointControllerState dof_state

# Security mode: true if security supervision is enabled, false otherwise.
bool secured_mode

# Security triggered: true if a security measure has been triggered, false otherwise.
bool security_triggered
