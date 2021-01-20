#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
    FPPControllerBase::FPPControllerBase(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info)
    {
        this->robot_info_ = robot_info;
    }
}