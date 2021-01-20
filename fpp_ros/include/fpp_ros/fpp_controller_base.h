#pragma once

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include <fpp_ros/data_classes/robot_info.h>

namespace fpp
{
    class FPPControllerBase
    {
        public:
            FPPControllerBase(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info);

            // GLAUBE DAS HIER BRAUCHE ICH NICHT ALS PURE VIRTUAL MACHEN
            /**
             * @brief This method is called after initializing the object.
             * This has to be implemented by the master and slave
             */
            // virtual void initialize() = 0;

            // ANDEREN NAMEN AUSDENKEN. AUCH IRGENDWAS MIT MAKE PLAN?
            /**
             * 
             * @brief This method is called when the planning should happen
             * This has to be imeplemented by the master and slave
             */
            // virtual void execute() = 0;

        protected:
            //! This is all the information that was read from the config file
            std::shared_ptr<fpp_data_classes::RobotInfo> robot_info_;

    };
}