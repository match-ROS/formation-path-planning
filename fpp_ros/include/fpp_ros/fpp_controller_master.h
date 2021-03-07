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
// #include <fpp_ros/footprint_generation/robot_footprint_ros.h>
// #include <fpp_ros/footprint_generation/formation_footprint_ros.h>
#include <fp_utils/geometry_info/minimal_enclosing_circle.h>
#include <fpp_ros/data_classes/ras_param_manager.h>

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

			// std::shared_ptr<footprint_generation::FormationFootprintRos> createFootprintObj(
			// 	std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> robot_info_list);

			void calcRobotPlans(const std::vector<geometry_msgs::PoseStamped> &formation_plan);

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
			//!
			std::shared_ptr<fpp_data_classes::RASParamManager> ras_param_manager_;

            // Process information
            
            //! Outline of the real formation that occures through amcl poses
            // std::shared_ptr<footprint_generation::FormationFootprintRos> real_formation_contour_;
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

            //! Topic to publish the plan of the formation. From this each robot has to calculate its own plan
            ros::Publisher formation_plan_pub_;

            //Timers

            //! Timer that periodically calculates and publishes the footprint of the formation
            ros::Timer footprint_timer_;
    };
}