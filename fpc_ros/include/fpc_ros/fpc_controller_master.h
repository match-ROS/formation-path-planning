#pragma once

#include "ros/ros.h"

#include <memory>
#include <vector>

#include <fpc_ros/fpc_controller_base.h>

namespace fpc
{
	class FPCControllerMaster : public fpc::FPCControllerBase
	{
		public:
			FPCControllerMaster(
				std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);

		private:
	};
}