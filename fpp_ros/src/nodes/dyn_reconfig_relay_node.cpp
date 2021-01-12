#include "ros/ros.h"
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>

#include <fpp_msgs/DynReconfigure.h>

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
    ros::ServiceServer dyn_reconfig_inflation_srv_server = nh.advertiseService("dyn_reconfig_inflation", dyn_reconfig_inflation_cb);

    while(ros::ok())
    {
        if(request_list.size() != 0)
        {
            fpp_msgs::DynReconfigure::Request request = request_list.back();
            request_list.pop_back();

            ROS_INFO("Dynamic reconfigure für robot in namespace: %s", request.robot_namespace.c_str());

            dynamic_reconfigure::Reconfigure dyn_reconfigure_msg;
            dynamic_reconfigure::DoubleParameter param_to_reconfig;
            param_to_reconfig.name = "inflation_radius";
            param_to_reconfig.value = request.new_inflation_radius;
            dyn_reconfigure_msg.request.config.doubles.push_back(param_to_reconfig);
            
            ROS_INFO("before call");

            ros::service::call(request.robot_namespace + "/move_base_flex/global_costmap/inflation/set_parameters",
                               dyn_reconfigure_msg.request,
                               dyn_reconfigure_msg.response);
            ROS_INFO("size: %i", dyn_reconfigure_msg.response.config.doubles.size());
            ROS_INFO("after call");

            ROS_INFO("Dynamic reconfigure end");
        }
        ros::spinOnce();
    }
    return 0;
}