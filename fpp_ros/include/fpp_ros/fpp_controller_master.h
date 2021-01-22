#pragma once

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include <fpp_ros/fpp_controller_base.h>

#include <fpp_ros/path_planner/minimal_enclosing_circle.h>

namespace fpp
{
    class FPPControllerMaster : public FPPControllerBase
    {
        public:
            FPPControllerMaster(std::shared_ptr<std::vector<fpp_data_classes::RobotInfo>> robot_info_list,
                                std::shared_ptr<fpp_data_classes::RobotInfo> robot_info,
                                ros::NodeHandle *nh);

            void execute() override;

        private:        
            void initServices();
            void initTopics();
            void initTimers();

            geometry_msgs::PoseWithCovarianceStampedConstPtr getAMCLPose(std::string robot_namespace);
            void getAMCLPose(std::string robot_namespace, Eigen::Vector2f &robot_pose, float &yaw);

            void updateFootprint();
            void publishFootprint();
            void footprintTimerCallback(const ros::TimerEvent& e);

            // Process information
            std::map<std::string, geometry_info::GeometryContour> robot_outline_list_;
            geometry_info::FormationContour formation_contour_;

            // Services and Topics

            // Because I was not able to dynamically reconfigure the costmap from this class
            // I had to create a relay node that would get a service (this one) and forward
            // it to the dynamic reconfigure server
            ros::ServiceClient dyn_rec_inflation_srv_client_;

            ros::Publisher formation_footprint_pub_;

            //Timers

            //! Timer that periodically calculates and publishes the footprint of the formation
            ros::Timer footprint_timer_;
    };
}