# Full timestamped NERO arm-status observation.
int64 source_timestamp_ns
int64 receipt_ros_timestamp_ns
int64 receipt_monotonic_ns
uint8 ctrl_mode
uint8 arm_status
uint8 mode_feedback
uint8 teach_status
uint8 motion_status
uint8 trajectory_num
uint32 error_code
bool[7] joint_angle_limit
bool[7] communication_status_joint
bool fresh
bool valid
