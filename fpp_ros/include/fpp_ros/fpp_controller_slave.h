#pragma once

#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
    class FPPControllerSlave : public FPPControllerBase
    {
        public:
            FPPControllerSlave(std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list,
                               std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info,
                               ros::NodeHandle &nh,
                               ros::NodeHandle &planner_nh);

            void execute(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) override;

        private:
			std::shared_ptr<nav_msgs::Path> master_plan_;

			ros::Subscriber master_plan_subscriber_;

			/**
             * @brief Helper method for initializing all topics
             * 
             */
            void initTopics() override;

			void masterPlanCb(const nav_msgs::Path::ConstPtr& master_plan);
    };
}