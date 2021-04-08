#pragma once

#include "ros/ros.h"

#include <memory>
#include <vector>
#include <map>

#include <fpc_ros/fpc_controller_base.h>
#include <fpp_msgs/NextTargetPoseCommand.h>

namespace fpc
{
	class FPCControllerMaster : public fpc::FPCControllerBase
	{
		public:
			FPCControllerMaster(
				std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);

			bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

		private:
			int pose_index_;
			float velocity_factor_;
			ros::Time next_target_pose_;			
			
			std::map<std::string, std::shared_ptr<ros::ServiceClient>> next_target_command_clt_list_;
			std::map<std::string, fpp_msgs::NextTargetPoseCommand::Response> saved_command_res_list_;



			void runController() override;

			/**
             * @brief Helper method for intializing all services
             * 
             */
            void initServices() override;


			#pragma region Callback Methods
			#pragma endregion

			#pragma region Helper Methods
			bool allSlavesReady();
			bool isSlaveReady(std::string robot_name);
			float getLargestDiff();
			// int getHighestPoseIndex();
			// int getLowestPoseIndex();
			// fpp_msgs::FPCRobotScaleInfo getHighestLinScaleValue(int next_target_pose);
			#pragma endregion
	};
}