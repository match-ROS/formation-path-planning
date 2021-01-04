#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <nav_core/base_global_planner.h>

#include <debug_helper/visualization_helper.h>

namespace fpp
{
    class FormationPathPlanner : public nav_core::BaseGlobalPlanner, public mbf_costmap_core::CostmapPlanner
    {
        public: 
            FormationPathPlanner();

        private:
            
    };
}