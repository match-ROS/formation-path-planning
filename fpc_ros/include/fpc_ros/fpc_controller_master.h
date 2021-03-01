#pragma once

#include "ros/ros.h"

#include <memory>
#include <vector>

#include <fpc_ros/fpc_controller_base.h>
#include <fpc_ros/data_classes/local_planner_robot_info.hpp>

namespace fpc
{
	class FPCControllerMaster : public fpc::FPCControllerBase
	{
		public:
			FPCControllerMaster(
				std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list,
				std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);

		private:
	};
}