echo "Sending to mur"
rsync --delete -avzhe ssh ~/catkin_ws_lurz/src rosmatch@mur:~/catkin_ws_lurz
ssh rosmatch@mur "cd ~/catkin_ws_lurz && catkin build formation_costmap fp_tests fp_utils fpc_ros fpp_gazebo fpp_launch fpp_msgs fpp_ros"