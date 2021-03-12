#include <fpc_ros/fpc_controller_master.h>

namespace fpc
{
	FPCControllerMaster::FPCControllerMaster(
		std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list,
		std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: FPCControllerBase(robot_info_list, robot_info, nh, controller_nh)
	{ }
}