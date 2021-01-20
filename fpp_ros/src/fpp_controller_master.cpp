#include <fpp_ros/fpp_controller_master.h>

namespace fpp
{
    FPPControllerMaster::FPPControllerMaster(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info) :
        FPPControllerBase(robot_info)
    {

    }
}