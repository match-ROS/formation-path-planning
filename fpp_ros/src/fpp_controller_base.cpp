#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
    FPPControllerBase::FPPControllerBase(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                                         fpp_data_classes::RobotInfo *&robot_info,
                                         ros::NodeHandle &nh,
                                         costmap_2d::Costmap2D *costmap)
        : robot_info_list_(robot_info_list), robot_info_(robot_info), nh_(nh), costmap_(costmap)
    {

    }
}