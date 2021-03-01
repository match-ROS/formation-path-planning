#pragma once

#include "ros/ros.h"

#include <fpc_ros/fpc_controller_base.h>

namespace fpc
{
	class FPCControllerSlave : public fpc::FPCControllerBase
	{
		public:
			FPCControllerSlave(
				std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list,
				std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);

		private:
	};
}