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

#include <fpp_ros/path_planner/splined_relaxed_a_star.h>
#include <fpp_ros/geometry_info/minimal_enclosing_circle.h>

namespace fpp
{
    class FPPControllerMaster : public FPPControllerBase
    {
        public:
            FPPControllerMaster(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                                fpp_data_classes::RobotInfo *&robot_info,
                                ros::NodeHandle &nh);

            void execute(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) override;

        private:      
            /**
             * @brief Helper method for intializing all services
             * 
             */
            void initServices();
            /**
             * @brief Helper method for initializing all topics
             * 
             */
            void initTopics();
            /**
             * @brief Helper method for initializing all timers
             * 
             */
            void initTimers();

            /**
             * @brief Method for getting the amcl pose of a specified robot. Methods waits until one amcl_pose msg is received.
             * 
             * @param robot_namespace Namespace the amcl node is located in
             * @return geometry_msgs::PoseWithCovarianceStampedConstPtr Pose of the robot that was retrieved
             */
            geometry_msgs::PoseWithCovarianceStampedConstPtr getAMCLPose(std::string robot_namespace);
            /**
             * @brief Helper method for getting the amcl pose of the specified robot.
             * Easier usability as it outputs the x/y position and yaw directly
             * 
             * @param robot_namespace Namespace the amcl node is located in
             * @param robot_pose This will be overwritten by the node. Will contain the x/y pose of the robot
             * @param yaw This will be overwritten by the node. Will contain the yaw of the robot
             */
            void getAMCLPose(std::string robot_namespace, Eigen::Vector2f &robot_pose, float &yaw);

            /**
             * @brief Update the footprint of the formation with the new positions of the individual robots
             * 
             */
            void updateFootprint();
            /**
             * @brief Publish the current footprint through the "formation_footprint" topic
             * 
             */
            void publishFootprint();
            /**
             * @brief Callback for the timer that triggers the publishing of the formation footprint
             * 
             * @param e TimerEvents handed to the callback 
             */
            void footprintTimerCallback(const ros::TimerEvent& e);

            // Process information
            
            //! List of all outlines of the individual robots
            std::map<std::string, geometry_info::RobotContour> robot_outline_list_;
            //! Outline of the formation
            geometry_info::FormationContour formation_contour_;

            // Services and Topics

            //! Because I was not able to dynamically reconfigure the costmap from this class
            //! I had to create a relay node that would get a service (this one) and forward
            //! it to the dynamic reconfigure server
            ros::ServiceClient dyn_rec_inflation_srv_client_;
            //! Topic to publish the footprint of the formation
            ros::Publisher formation_footprint_pub_;

            //Timers

            //! Timer that periodically calculates and publishes the footprint of the formation
            ros::Timer footprint_timer_;
    };
}