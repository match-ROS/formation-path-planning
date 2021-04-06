#pragma once

#include "ros/ros.h"

#include <memory>
#include <vector>
#include <map>

#include <fpc_ros/fpc_controller_base.h>
#include <fpp_msgs/FPCRobotScaleInfo.h>
#include <fpp_msgs/FPCVelScaleInfo.h>

namespace fpc
{
	class FPCControllerMaster : public fpc::FPCControllerBase
	{
		public:
			FPCControllerMaster(
				std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);

			void run_controller() override;

		private:
			ros::ServiceServer fpc_vel_scale_info_srv_;

			std::map<std::string, fpp_msgs::FPCRobotScaleInfo> robot_scale_info_list_;

			/**
             * @brief Helper method for intializing all services
             * 
             */
            void initServices() override;

			#pragma region Callback Methods
			bool onFPCVelScaleInfo(fpp_msgs::FPCVelScaleInfo::Request &req,
								   fpp_msgs::FPCVelScaleInfo::Response &res);
			#pragma endregion

			#pragma region Helper Methods
			int getHighestPoseIndex();
			int getLowestPoseIndex();
			fpp_msgs::FPCRobotScaleInfo getHighestLinScaleValue(int next_target_pose);
			#pragma endregion
	};
}