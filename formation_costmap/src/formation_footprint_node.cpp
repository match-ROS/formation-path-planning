#include "ros/ros.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <string>

#include <formation_costmap/formation_costmap_params.h>
#include <formation_costmap/formation_footprint_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_footprint_node");

	std::string costmap_name = "formation_costmap";

    ros::NodeHandle nh = ros::NodeHandle();
	ros::NodeHandle costmap_nh = ros::NodeHandle("~" + costmap_name);

	// Init all topics, services and actions
	//! Topic to publish the footprint of the formation
	ros::Publisher formation_footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10, true);

	// Get parameter from the yaml file for the formation costmap
	formation_costmap::FormationCostmapParamManager fc_param_manager =
		formation_costmap::FormationCostmapParamManager(nh, costmap_nh);
	fc_param_manager.getParams(costmap_name);
	std::shared_ptr<formation_costmap::FormationCostmapParams> fc_params;
	fc_params = fc_param_manager.getFormationCostmapParams();

	// Create the formation footprint object by params
	std::shared_ptr<formation_costmap::FormationFootprintRos> formation_footprint =
		std::make_shared<formation_costmap::FormationFootprintRos>();
	for (const std::shared_ptr<formation_costmap::FCRobotParams> &robot_info_it :
		 fc_params->formation_robot_params)
	{
		std::shared_ptr<formation_costmap::RobotFootprintRos> robot_contour =
			std::make_shared<formation_costmap::RobotFootprintRos>(nh,
																   robot_info_it->robot_name,
																   robot_info_it->robot_namespace,
																   robot_info_it->robot_pose_topic);

		for (Eigen::Vector2f corner : robot_info_it->robot_outline)
		{
			robot_contour->addContourCornerGeometryCS(corner);
		}
		robot_contour->createContourEdges();
		formation_footprint->addRobotToFormation(robot_contour);
	}

	while(ros::ok())
    {
		formation_footprint_pub.publish(formation_footprint->getFormationFootprint());
		
        ros::spinOnce();
		ros::Duration(0.1).sleep();
    }
    return 0;
}