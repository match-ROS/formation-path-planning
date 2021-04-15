# Very important to log the data with the right timestamp!
time stamp

# Info about the current pose index
int32 pose_index

# Current values
geometry_msgs/Pose2D current_pose
geometry_msgs/Twist current_vel

# Target values
geometry_msgs/Pose2D target_pose
geometry_msgs/Twist target_vel

# Diff values
geometry_msgs/Pose2D current_to_target_pose_diff
geometry_msgs/Twist current_to_target_vel_diff

# Aditional pose values
geometry_msgs/Pose2D ground_truth_pose
geometry_msgs/Pose2D odom_pose

# Interesting info
geometry_msgs/Pose2D current_to_ground_truth_pose_diff

float32 velocity_factor