#include "ros/ros.h"
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <costmap_2d/InflationPluginConfig.h>

#include <std_srvs/Empty.h>

ros::ServiceServer server;
ros::ServiceClient temp_;

bool empty(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("SERVICE CALL");
    ROS_INFO("1");
    dynamic_reconfigure::Reconfigure rec;
    ROS_INFO("2");
    dynamic_reconfigure::DoubleParameter param_to_reconfig;
    ROS_INFO("3");
    param_to_reconfig.name = "inflation_radius";
    param_to_reconfig.value = 0.8;
    ROS_INFO("5");
    rec.request.config.doubles.push_back(param_to_reconfig);
    ROS_INFO("size: %i", rec.request.config.doubles.size());
    ROS_INFO("6");
    //bool b = temp_.call(rec);
    // ROS_INFO("%i", b);
    ros::service::call("/robot1/move_base_flex/global_costmap/inflation/set_parameters", rec.request, rec.response);
    ROS_INFO("7");
    ROS_INFO("size: %i", rec.response.config.doubles.size());
    ROS_INFO("LIST:");
    for(dynamic_reconfigure::DoubleParameter param: rec.response.config.doubles)
    {
        ROS_INFO("name: %s, value: %f", param.name.c_str(), param.value);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_dyn_rec");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("test_service", empty);
    ros::ServiceClient temp_ = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/robot1/move_base_flex/global_costmap/inflation/set_parameters");
    temp_.waitForExistence();

    // ROS_INFO("SERVICE CALL");
    // ROS_INFO("1");
    // dynamic_reconfigure::Reconfigure rec;
    // ROS_INFO("2");
    // dynamic_reconfigure::DoubleParameter param_to_reconfig;
    // ROS_INFO("3");
    // param_to_reconfig.name = "inflation_radius";
    // param_to_reconfig.value = 5.0;
    // ROS_INFO("5");
    // rec.request.config.doubles.push_back(param_to_reconfig);
    // ROS_INFO("size: %i", rec.request.config.doubles.size());
    // ROS_INFO("6");
    // bool b = temp_.call(rec);
    // ROS_INFO("%i", b);
    // //ros::service::call("/robot1/move_base_flex/global_costmap/inflation/set_parameters", reconf_req, reconf_res);
    // ROS_INFO("7");
    // ROS_INFO("size: %i", rec.response.config.doubles.size());
    // ROS_INFO("LIST:");
    // for(dynamic_reconfigure::DoubleParameter param: rec.response.config.doubles)
    // {
    //     ROS_INFO("name: %s, value: %f", param.name.c_str(), param.value);
    // }
    ros::spin();
    return 0;
}