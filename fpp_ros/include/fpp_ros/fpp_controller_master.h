#pragma once

#include <fpp_ros/fpp_controller_base.h>

#include <actionlib/client/simple_action_client.h>

#include <mbf_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <fpp_msgs/GetRobotPlan.h>
#include <fpp_msgs/DynReconfigure.h>
#include <fpp_msgs/FormationFootprintInfo.h>

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
								const fpp_data_classes::FPPControllerParams &fpp_controller_params,
								ros::NodeHandle &nh,
								ros::NodeHandle &planner_nh);

            void execute(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) override;

        private:     
            #pragma region Process Info
            //! Replace this in the future with an interface pointer. Object that plans the initial path.
            path_planner::SplinedRelaxedAStar initial_path_planner_;
			#pragma endregion

			#pragma region Parameters
			//! This parameter contains the name of the used planner for generating the initial plan
            std::string used_formation_planner_;
			//! This object manages all params for the Relaxed A Star planner
			std::shared_ptr<fpp_data_classes::RASParamManager> ras_param_manager_;
			#pragma endregion

            #pragma region Topics/Services/Actions
			//! Topic to publish the meta data of the formation plan to
			ros::Publisher formation_plan_meta_data_pub_;
            //! Because I was not able to dynamically reconfigure the costmap from this class
            //! I had to create a relay node that would get a service (this one) and forward
            //! it to the dynamic reconfigure server
            ros::ServiceClient dyn_rec_inflation_srv_client_;
			//! Get information about the formation. Centre, minimal radius, etc.
			ros::ServiceClient get_footprint_info_srv_client_;
			//! List that contains an action client linked to all slave robots in the formation
			std::vector<std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>> slave_move_base_as_list_;

            //! Topic to publish the plan of the formation. From this each robot has to calculate its own plan
            ros::Publisher formation_plan_pub_;
			#pragma endregion

			void initServices() override;
            void initTopics() override;
			void initActions() override;
            void initTimers() override;

			/**
             * @brief Read params from config file
             * 
             */
			void readRASParams(std::string formation_planner_name);

            /**
             * @brief Call the dynamic reconfigure relay node to reconfigure the costmap inflation
             * 
             */
            void callDynamicCostmapReconfigure(float min_formation_circle_radius);
    };
}