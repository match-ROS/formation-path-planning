# Very important to log the data with the right timestamp!
time stamp

# Current values
geometry_msgs/Pose2D current_pose
geometry_msgs/Twist current_vel

# Target values
geometry_msgs/Pose2D target_pose
geometry_msgs/Twist target_vel

# Aditional pose values
geometry_msgs/Pose ground_truth_pose
geometry_msgs/Pose odom_pose
geometry_msgs/Pose amcl_pose

# Diff values
geometry_msgs/Pose2D current_to_target_pose_diff
geometry_msgs/Twist current_to_target_vel_diff

# Interesting info
geometry_msgs/Pose2D current_to_amcl_pose_diff
geometry_msgs/Pose2D current_to_ground_truth_pose_diff
geometry_msgs/Pose2D amcl_to_ground_truth_diff