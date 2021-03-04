#pragma once

#include <fpp_ros/fpp_controller_base.h>

#include <actionlib/client/simple_action_client.h>

#include <mbf_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <fpp_msgs/GetRobotPlan.h>
#include <fpp_msgs/DynReconfigure.h>

#include <fpp_ros/path_planner/splined_relaxed_a_star.h>
#include <fpp_ros/footprint_generation/robot_footprint_ros.h>
#include <fpp_ros/footprint_generation/formation_footprint_ros.h>
#include <fpp_ros/geometry_info/minimal_enclosing_circle.h>

#include <Eigen/Dense>
#include <map>

namespace fpp
{
    class FPPControllerMaster : public FPPControllerBase
    {
        public:
            FPPControllerMaster(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
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

            void initServices() override;
            void initTopics() override;
			void initActions() override;
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

			void calcRobotPlans(const std::vector<geometry_msgs::PoseStamped> &formation_plan);

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

			bool getRobotPlanCb(fpp_msgs::GetRobotPlan::Request &req, fpp_msgs::GetRobotPlan::Response &res);

            // Parameter information
            //! This parameter contains the name of the used planner for generating the initial plan
            std::string used_formation_planner_;

            // Process information
            
            //! List of all outlines of the individual robots
            std::map<std::string, footprint_generation::RobotFootprintRos> robot_outline_list_;
            //! Outline of the real formation that occures through amcl poses
            footprint_generation::FormationFootprintRos real_formation_contour_;
			//! Outline of the formation of everything is ideal
            // footprint_generation::FormationFootprintRos target_formation_contour_;
            //! Centre of the formation
            Eigen::Vector2f formation_centre_;
            //! Minimal circle around the formation to change the costmap inflation
            geometry_info::MinimalEnclosingCircle formation_enclosing_circle_;
            //! Replace this in the future with an interface pointer. Object that plans the initial path.
            path_planner::SplinedRelaxedAStar initial_path_planner_;
			//! List of the calculated plans for each robot
			std::map<std::string, std::vector<geometry_msgs::PoseStamped>> robot_plan_list_;

            // Services and Topics

            //! Because I was not able to dynamically reconfigure the costmap from this class
            //! I had to create a relay node that would get a service (this one) and forward
            //! it to the dynamic reconfigure server
            ros::ServiceClient dyn_rec_inflation_srv_client_;
			//!
			ros::ServiceServer get_robot_plan_srv_server_;
			//! List that contains an action client linked to all slave robots in the formation
			std::vector<std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>> slave_move_base_as_list_;

            //! Topic to publish the footprint of the formation
            ros::Publisher formation_footprint_pub_;
            //! Topic to publish the plan of the formation. From this each robot has to calculate its own plan
            ros::Publisher formation_plan_pub_;

            //Timers

            //! Timer that periodically calculates and publishes the footprint of the formation
            ros::Timer footprint_timer_;
    };
}