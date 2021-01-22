#pragma once

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
    class FPPControllerSlave : public FPPControllerBase
    {
        public:
            FPPControllerSlave(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                               fpp_data_classes::RobotInfo *&robot_info,
                               ros::NodeHandle &nh);

            void execute() override {};

        private:
    };
}