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
            FPPControllerMaster(std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list,
                                std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info,
                                ros::NodeHandle &nh,
                                ros::NodeHandle &planner_nh);

            void initialize(std::string planner_name,
                            costmap_2d::Costmap2D *costmap,
                            std::string global_frame) override;

            void execute(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) override;

        private:     
            /**
             * @brief Read params from config file
             * 
             */
            void readParams(std::string name);
            /**
             * @brief Helper method for intializing all services
             * 
             */
            void initServices() override;
            /**
             * @brief Helper method for initializing all topics
             * 
             */
            void initTopics() override;
            /**
             * @brief Helper method for initializing all timers
             * 
             */
            void initTimers() override;

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
             * @brief Call the dynamic reconfigure relay node to reconfigure the costmap inflation
             * 
             */
            void callDynamicCostmapReconfigure();
            /**
             * @brief Callback for the timer that triggers the publishing of the formation footprint
             * 
             * @param e TimerEvents handed to the callback 
             */
            void footprintTimerCallback(const ros::TimerEvent& e);

            // Parameter information
            //! This parameter contains the name of the used planner for generating the initial plan
            std::string used_formation_planner_;

            // Process information
            
            //! List of all outlines of the individual robots
            std::map<std::string, geometry_info::RobotContour> robot_outline_list_;
            //! Outline of the real formation that occures through amcl poses
            geometry_info::FormationContour real_formation_contour_;
			//! Outline of the formation of everything is ideal
            geometry_info::FormationContour target_formation_contour_;
            //! Centre of the formation
            Eigen::Vector2f formation_centre_;
            //! Minimal circle around the formation to change the costmap inflation
            geometry_info::MinimalEnclosingCircle formation_enclosing_circle_;
            //! Replace this in the future with an interface pointer. Object that plans the initial path.
            path_planner::SplinedRelaxedAStar initial_path_planner_;

            // Services and Topics

            //! Because I was not able to dynamically reconfigure the costmap from this class
            //! I had to create a relay node that would get a service (this one) and forward
            //! it to the dynamic reconfigure server
            ros::ServiceClient dyn_rec_inflation_srv_client_;
            //! Topic to publish the footprint of the formation
            ros::Publisher formation_footprint_pub_;
            //! Topic to publish the plan of the formation. From this each robot has to calculate its own plan
            ros::Publisher formation_plan_pub_;

            //Timers

            //! Timer that periodically calculates and publishes the footprint of the formation
            ros::Timer footprint_timer_;
    };
}