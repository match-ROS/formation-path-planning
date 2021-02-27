#include "ros/ros.h"

#include <vector>
#include <string>

#include <fp_utils/bezier_splines/quintic_bezier_spline.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_reconfig_relay_node");
    ros::NodeHandle nh;
    
	bezier_splines::QuinticBezierSplines spline = bezier_splines::QuinticBezierSplines();

    return 0;
}