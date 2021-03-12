#pragma once

#include <fpp_ros/fpp_controller_base.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <fpp_msgs/GetRobotPlan.h>

#include <fpp_ros/data_classes/fpp_controller_param.h>

namespace fpp
{
    class FPPControllerSlave : public FPPControllerBase
    {
        public:
			FPPControllerSlave(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
							   const fpp_data_classes::FPPControllerParams &fpp_controller_params,
							   ros::NodeHandle &nh,
							   ros::NodeHandle &planner_nh);

			void execute(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) override;

        private:
			std::shared_ptr<nav_msgs::Path> formation_plan_;

			// ros::ServiceClient get_robot_plan_srv_client_;

			#pragma region Topics/Services/Actions
			ros::Subscriber formation_plan_sub_;
			#pragma endregion

			void initTopics() override;

			void getFormationPlanCb(nav_msgs::Path formation_plan);
    };
}