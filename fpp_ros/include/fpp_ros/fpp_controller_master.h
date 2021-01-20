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
    class FPPControllerMaster : public FPPControllerBase
    {
        public:
            FPPControllerMaster(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info);

        private:

    };
}