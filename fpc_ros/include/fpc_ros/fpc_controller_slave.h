#pragma once

#include "ros/ros.h"

#include <fpc_ros/fpc_controller_base.h>

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

			void run_controller() override;
	};
}