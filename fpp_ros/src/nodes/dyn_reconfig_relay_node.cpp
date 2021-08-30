#include "ros/ros.h"
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

#include <fpp_msgs/DynReconfigure.h>
#include <fpp_msgs/FormationFootprintInfo.h>
#include <geometry_msgs/PolygonStamped.h>

#include <vector>
#include <string>

std::string ns;
ros::ServiceServer dyn_reconfig_inflation_srv_server;
ros::ServiceClient dyn_reconfig_srv_client;

std::vector<fpp_msgs::DynReconfigure::Request> request_list;

bool dyn_reconfig_inflation_cb(fpp_msgs::DynReconfigure::Request &req, fpp_msgs::DynReconfigure::Response &res)
{
    request_list.push_back(req);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_reconfig_relay_node");
    ros::NodeHandle nh;
    ros::ServiceServer dyn_reconfig_inflation_srv_server = nh.advertiseService("/dyn_reconfig_inflation", dyn_reconfig_inflation_cb);
	ros::ServiceClient formation_info_client = nh.serviceClient<fpp_msgs::FormationFootprintInfo>("/robot0/move_base_flex/footprint_info");
	formation_info_client.waitForExistence();

    while(ros::ok())
    {
        if(request_list.size() != 0)
        {
			// Dynamic reconfigure for the inflation radius
            fpp_msgs::DynReconfigure::Request request = request_list.back();
            request_list.pop_back();

            ROS_INFO("Dynamic reconfigure für robot in namespace: %s", request.robot_namespace.c_str());

            dynamic_reconfigure::Reconfigure dyn_reconfigure_inflation_msg;
            dynamic_reconfigure::DoubleParameter inflation_reconfig;
            inflation_reconfig.name = "inflation_radius";
            inflation_reconfig.value = request.new_inflation_radius;
            ROS_INFO_STREAM("New inflation radius: " << request.new_inflation_radius);
            dyn_reconfigure_inflation_msg.request.config.doubles.push_back(inflation_reconfig);
            
            // ROS_INFO("before call");

			ROS_INFO_STREAM(request.robot_namespace << "/move_base_flex/global_costmap/inflation/set_parameters");
            ros::service::call(request.robot_namespace + "/move_base_flex/global_costmap/inflation/set_parameters",
                               dyn_reconfigure_inflation_msg.request,
                               dyn_reconfigure_inflation_msg.response);

			// Dynamic reconfigure for the footprint
			fpp_msgs::FormationFootprintInfo info;
			formation_info_client.call(info);

			dynamic_reconfigure::Reconfigure dyn_reconfigure_footprint_msg;
            dynamic_reconfigure::StrParameter footprint_reconfig;
			footprint_reconfig.name = "footprint";
			footprint_reconfig.value = "[";
			for(int counter = 0; counter < info.response.formation_footprint.points.size(); counter++)
			{
				footprint_reconfig.value = footprint_reconfig.value + "[" + std::to_string(info.response.formation_footprint.points[counter].x) + "," + std::to_string(info.response.formation_footprint.points[counter].y) + "]";
				if(counter < info.response.formation_footprint.points.size() - 1)
				{
					footprint_reconfig.value = footprint_reconfig.value + ",";
				}
			}
			footprint_reconfig.value = footprint_reconfig.value + "]";
			dyn_reconfigure_footprint_msg.request.config.strs.push_back(footprint_reconfig);

			ros::service::call(request.robot_namespace + "/move_base_flex/global_costmap/set_parameters",
                               dyn_reconfigure_footprint_msg.request,
                               dyn_reconfigure_footprint_msg.response);

            ROS_INFO("Dynamic reconfigure end");
        }
        ros::spinOnce();
    }
    return 0;
}