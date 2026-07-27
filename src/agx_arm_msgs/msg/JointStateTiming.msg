# Provenance for one JointState snapshot published on feedback/joint_states.
# header.stamp is the ROS-clock assembly timestamp and exactly matches the
# corresponding sensor_msgs/JointState header.
std_msgs/Header header

# Cached SocketCAN receive timestamp previously used as JointState.header.
builtin_interfaces/Time position_hardware_stamp

# SocketCAN receive timestamp for each arm motor state used to populate
# velocity and effort. The array follows the arm joint order (six or seven).
builtin_interfaces/Time[] motor_hardware_stamps
