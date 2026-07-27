# One independently timestamped SDK motor-state observation.
uint8 joint_index
int64 sdk_timestamp_ns
int64 driver_receipt_ros_timestamp_ns
int64 driver_receipt_monotonic_ns
float64 position_rad
float64 velocity_rad_s
float64 current_a
float64 torque_nm
bool fresh
bool valid
