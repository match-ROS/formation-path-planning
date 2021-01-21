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
            FPPControllerSlave(std::shared_ptr<std::vector<fpp_data_classes::RobotInfo>> robot_info_list,
                               std::shared_ptr<fpp_data_classes::RobotInfo> robot_info,
                               std::shared_ptr<ros::NodeHandle> nh);

            void execute() override {};

        private:
    };
}