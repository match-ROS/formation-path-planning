# Name of the robot to identify the info
string robot_name

# Current pose on the path the robot should be located at
int32 current_pose_index
# Next pose on the path the robot tries to reach
int32 next_target_pose_index
# Euclidean distance from the current position of the robot to the selected target pose
float32 distance_to_next_target

# Scaling that was used for the lin vel to get the max allowed (if lin vel is above max)
float32 lin_vel_scale
# Scaling that was used for the rot vel to get the max allowed (if rot vel is above max)
float32 rot_vel_scale