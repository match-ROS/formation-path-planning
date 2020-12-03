#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <nav_core/base_global_planner.h>

namespace advanced_dijkstra
{
    class AdvancedDijkstra : public nav_core::BaseGlobalPlanner, public mbf_costmap_core::CostmapPlanner
    {
        public:
            AdvancedDijkstra();
            AdvancedDijkstra(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    };
}