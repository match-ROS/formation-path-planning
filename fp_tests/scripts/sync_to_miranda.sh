echo "Sending to miranda"
rsync --delete -avzhe ssh ~/catkin_ws_lurz/src rosmatch@miranda:~/catkin_ws_lurz
ssh rosmatch@miranda "cd ~/catkin_ws_lurz && catkin build formation_costmap fp_tests fp_utils fpc_ros fpp_gazebo fpp_launch fpp_msgs fpp_ros"