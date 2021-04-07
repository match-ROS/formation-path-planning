#pragma once

#include "ros/ros.h"

#include <fpc_ros/fpc_controller_base.h>
#include <fpp_msgs/NextTargetPoseCommand.h>

namespace fpc
{
	class FPCControllerSlave : public fpc::FPCControllerBase
	{
		public:
			FPCControllerSlave(
				std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);

		private:
			ros::ServiceServer next_target_command_srv_;
			fpp_msgs::NextTargetPoseCommand::Request saved_target_cmd_req_;

			/**
             * @brief Helper method for intializing all services
             * 
             */
            void initServices() override;

			void runController() override;

			#pragma region Callback Methods
			bool onNextTargetCommand(fpp_msgs::NextTargetPoseCommand::Request &req,
									 fpp_msgs::NextTargetPoseCommand::Response &res);

			#pragma endregion
	};
}