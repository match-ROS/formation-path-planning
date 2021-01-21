#include <fpp_ros/fpp_controller_slave.h>

namespace fpp
{
    FPPControllerSlave::FPPControllerSlave(std::shared_ptr<std::vector<fpp_data_classes::RobotInfo>> robot_info_list,
                                           std::shared_ptr<fpp_data_classes::RobotInfo> robot_info,
                                           std::shared_ptr<ros::NodeHandle> nh)
        : FPPControllerBase(robot_info_list, robot_info, nh)
    {
        
    }

}