#include <fpc_ros/fpc_controller_master.h>

namespace fpc
{
	FPCControllerMaster::FPCControllerMaster(
		std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: FPCControllerBase(fpc_param_info, nh, controller_nh)
	{ }
}