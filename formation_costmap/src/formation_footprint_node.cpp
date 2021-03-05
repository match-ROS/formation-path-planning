#include "ros/ros.h"

#include <vector>
#include <string>

std::string ns;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_footprint_node");

	std::string costmap_name = "formation_costmap";

    ros::NodeHandle nh = ros::NodeHandle();
	ros::NodeHandle costmap_nh = ros::NodeHandle("~" + costmap_name);

	

    while(ros::ok())
    {
        
        ros::spinOnce();
    }
    return 0;
}