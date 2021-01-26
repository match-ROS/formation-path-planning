#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
    FPPControllerBase::FPPControllerBase(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                                         fpp_data_classes::RobotInfo *&robot_info,
                                         ros::NodeHandle &nh)
        : robot_info_list_(robot_info_list), robot_info_(robot_info), nh_(nh)
    {

    }

    void FPPControllerBase::initialize(std::string planner_name,
                                       costmap_2d::Costmap2D *costmap,
                                       std::string global_frame)
    {
        this->planner_name_ = planner_name;
        this->costmap_ = costmap;
        this->global_frame_ = global_frame;
    }
}