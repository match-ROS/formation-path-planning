#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace fpp_data_classes
{
    struct RobotInfo
    {
        std::string robot_name;
        std::string robot_namespace;
        bool fpp_master;
        Eigen::Vector2f offset;
        std::vector<Eigen::Vector2f> robot_outline;
    };
}