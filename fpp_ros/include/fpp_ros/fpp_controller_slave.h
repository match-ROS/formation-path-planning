#pragma once

#include <fpp_msgs/GetRobotPlan.h>

#include <fpp_ros/fpp_controller_base.h>

#include <fpp_ros/data_classes/robot_info.h>
#include <fpp_ros/data_classes/fpp_param_manager.h>

namespace fpp
{
    class FPPControllerSlave : public FPPControllerBase
    {
        public:
			FPPControllerSlave(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
							   ros::NodeHandle &nh,
							   ros::NodeHandle &planner_nh);

			void execute(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) override;

        private:
			std::shared_ptr<nav_msgs::Path> formation_plan_;
			std::shared_ptr<nav_msgs::Path> master_plan_;

			ros::ServiceClient get_robot_plan_srv_client_;

			ros::Subscriber master_plan_subscriber_;

			/**
             * @brief Helper method for initializing all topics
             * 
             */
            void initTopics() override;
			/**
             * @brief Helper method for intializing all services
             * 
             */
            void initServices() override;

			void formationPlanCb(const nav_msgs::Path::ConstPtr& formation_plan);
			void masterPlanCb(const nav_msgs::Path::ConstPtr& master_plan);
    };
}