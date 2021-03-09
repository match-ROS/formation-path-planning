#pragma once

#include <costmap_2d/costmap_2d.h>

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace fpp_data_classes
{
    struct FPPControllerParams
    {
		FPPControllerParams(std::string path_planner_name,
							costmap_2d::Costmap2D *costmap,
							std::string global_frame)
		{
			this->path_planner_name = path_planner_name;
			this->costmap = costmap;
			this->global_frame = global_frame;
		}

		//! Name of the path planner this controller is used in
        std::string path_planner_name;
		//! Pointer to the costmap of the map
        costmap_2d::Costmap2D *costmap;
		//! Name of the global frame where to robot is located in
		std::string global_frame;
    };
}