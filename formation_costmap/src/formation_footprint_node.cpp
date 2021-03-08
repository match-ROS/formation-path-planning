#include "ros/ros.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <string>

#include <formation_costmap/formation_costmap_params.h>
#include <formation_costmap/formation_footprint_ros.h>
#include <fpp_msgs/FormationFootprintInfo.h>
#include <fpp_msgs/RobotOutline.h>

std::shared_ptr<formation_costmap::FormationFootprintRos> formation_footprint;
std::shared_ptr<formation_costmap::FormationCostmapParams> fc_params;

bool formation_footprint_info_cb(fpp_msgs::FormationFootprintInfo::Request &req,
								 fpp_msgs::FormationFootprintInfo::Response &res)
{
	Eigen::Vector2f formation_centre_world_cs;
	formation_centre_world_cs = formation_footprint->calcCentroidWorldCS();
	res.formation_centre.x = formation_centre_world_cs[0];
	res.formation_centre.y = formation_centre_world_cs[1];
	res.formation_centre.theta = 0.0;

	res.minimal_encloring_circle_radius = formation_footprint->calcMinimalEnclosingCircleRadius();

	return true;
}

bool robot_outline_cb(fpp_msgs::RobotOutline::Request &req,
					  fpp_msgs::RobotOutline::Response &res)
{
	auto it = std::find_if(fc_params->formation_robot_params.begin(),
						   fc_params->formation_robot_params.end(),
						   [&req](std::shared_ptr<formation_costmap::FCRobotParams> &robot_params) { return req.robot_name == robot_params->robot_name; });
	
	if (it != fc_params->formation_robot_params.end())
	{
		geometry_msgs::PolygonStamped robot_footprint_msg;
        robot_footprint_msg.header.frame_id = "map";
        robot_footprint_msg.header.stamp = ros::Time::now();

        std::vector<Eigen::Vector2f> robot_corner_points = it->get()->robot_outline;

        for(Eigen::Vector2f corner: robot_corner_points)
        {
            geometry_msgs::Point32 corner_point;
            corner_point.x = corner[0];
            corner_point.y = corner[1];
            corner_point.z = 0.0;
            robot_footprint_msg.polygon.points.push_back(corner_point);
        }

		res.outline = robot_footprint_msg;
		return true;
	}
	return false; // Robot name was not found
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_footprint_node");

	std::string costmap_name = "formation_costmap";

    ros::NodeHandle nh = ros::NodeHandle();
	ros::NodeHandle costmap_nh = ros::NodeHandle("~" + costmap_name);

	// Init all topics, services and actions
	//! Topic to publish the footprint of the formation
	ros::Publisher formation_footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10, true);
	ros::ServiceServer formation_footprint_info_srv = nh.advertiseService("footprint_info",
																		  &formation_footprint_info_cb);
	ros::ServiceServer robot_outline_srv = nh.advertiseService("robot_outline",
															   &robot_outline_cb);

	// Get parameter from the yaml file for the formation costmap
	formation_costmap::FormationCostmapParamManager fc_param_manager =
		formation_costmap::FormationCostmapParamManager(nh, costmap_nh);
	fc_param_manager.getParams(costmap_name);
	fc_params = fc_param_manager.getFormationCostmapParams();

	// Create the formation footprint object by params
	formation_footprint = std::make_shared<formation_costmap::FormationFootprintRos>();
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