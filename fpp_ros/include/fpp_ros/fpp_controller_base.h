#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include <fpp_msgs/DynReconfigure.h>
#include <fpp_ros/data_classes/robot_info.h>

#include <fpp_ros/geometry_info/geometry_contour.h>
#include <fpp_ros/geometry_info/formation_contour.h>

namespace fpp
{
    class FPPControllerBase
    {
        public:
            FPPControllerBase(std::shared_ptr<std::vector<fpp_data_classes::RobotInfo>> robot_info_list,
                              std::shared_ptr<fpp_data_classes::RobotInfo> robot_info,
                              std::shared_ptr<ros::NodeHandle> nh);

            // ANDEREN NAMEN AUSDENKEN. AUCH IRGENDWAS MIT MAKE PLAN?
            /**
             * 
             * @brief This method is called when the planning should happen
             * This has to be imeplemented by the master and slave
             */
            virtual void execute() = 0;

        protected:
            //! NodeHandle from the node that initializes the fpp controller classes
            std::shared_ptr<ros::NodeHandle> nh_;

            //! This is all the information that was read from the config file about each robot
            std::shared_ptr<std::vector<fpp_data_classes::RobotInfo>> robot_info_list_;

            //! This points to the object in the robot_info_list_ that contains the information about this robot
            std::shared_ptr<fpp_data_classes::RobotInfo> robot_info_;

    };
}