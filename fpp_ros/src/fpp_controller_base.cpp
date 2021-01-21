#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
    FPPControllerBase::FPPControllerBase(std::shared_ptr<std::vector<fpp_data_classes::RobotInfo>> robot_info_list,
                                         std::shared_ptr<fpp_data_classes::RobotInfo> robot_info,
                                         std::shared_ptr<ros::NodeHandle> nh)
    {
        this->nh_ = nh;
        this->robot_info_list_ = robot_info_list;
        this->robot_info_ = robot_info;
    }
}