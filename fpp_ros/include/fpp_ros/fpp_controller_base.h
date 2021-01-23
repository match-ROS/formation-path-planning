#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include <fpp_msgs/DynReconfigure.h>
#include <fpp_ros/data_classes/robot_info.h>

#include <fpp_ros/geometry_info/geometry_contour.h>
#include <fpp_ros/geometry_info/robot_contour.h>
#include <fpp_ros/geometry_info/formation_contour.h>

namespace fpp
{
    class FPPControllerBase
    {
        public:
            /**
             * @brief Constructor for the BaseObject of the FPPController. 
             * This stores basic information that is needed for the FPPController
             * 
             * @param robot_info_list Information about all robots in the formation
             * @param robot_info Reference to pointer that points to element in the robot_info_list that defines the information about the robot this controller is running on.
             * @param nh Nodehandle for topics/services and actions
             */
            FPPControllerBase(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                              fpp_data_classes::RobotInfo *&robot_info,
                              ros::NodeHandle &nh);

            /**
             * 
             * @brief This method is called when the planning should happen
             * This has to be implemented by the master and slave as it is pure virtual
             */
            virtual void execute() = 0;

        protected:
            //! NodeHandle from the node that initializes the fpp controller classes
            ros::NodeHandle &nh_;

            //! This is all the information that was read from the config file about each robot
            std::list<fpp_data_classes::RobotInfo> &robot_info_list_;

            //! This points to the object in the robot_info_list_ that contains the information about this robot
            fpp_data_classes::RobotInfo *&robot_info_;

    };
}