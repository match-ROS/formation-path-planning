# Contains all information that is needed to reconstruct the Bezier spline
# Start and End pose that the spline is required to got through
geometry_msgs/Point start_pose
geometry_msgs/Point end_pose

# Direction the spline should have at the start and at the end
geometry_msgs/Point start_tangent
geometry_msgs/Point end_tangent

# Magnitude the tangent vectors have at the start and end. This is important for the curvation of the spline
float32 start_tangent_magnitude
float32 end_tangent_magnitude

