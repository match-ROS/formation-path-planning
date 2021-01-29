#include <fpp_ros/fpp_controller_slave.h>

namespace fpp
{
    FPPControllerSlave::FPPControllerSlave(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                                           fpp_data_classes::RobotInfo *&robot_info,
                                           ros::NodeHandle &nh,
                                           ros::NodeHandle &planner_nh)
        : FPPControllerBase(robot_info_list, robot_info, nh, planner_nh)
    {
        
    }

    void FPPControllerSlave::execute(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     std::vector<geometry_msgs::PoseStamped> &plan)
    {
    }
}