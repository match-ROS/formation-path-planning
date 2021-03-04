#pragma once

#include "ros/ros.h"

#include <fpp_ros/data_classes/path_planner_types.h>

namespace fpp_data_classes
{
	struct RASParams
	{
		// Settings for RelaxedAStar planner
		float default_tolerance;
		fpp_data_classes::NeighborType neighbor_type;
		int free_cell_threshold;
		
		// Settings to make start and end easier for the mobile robots
		float start_straight_distance;
		float end_straight_distance;

		// Settings for generating points with Bezier Splines
		int control_point_distance;
		int planning_points_per_spline;
		float minimal_curve_radius;
	};

	class RASParamManager
	{
		public:
			RASParamManager(ros::NodeHandle &nh, ros::NodeHandle &planner_nh);
		
			void getParams(std::string formation_planner_name);

			#pragma region Getter/Setter
			fpp_data_classes::RASParams getRASParams();
			#pragma endregion

		private:
			ros::NodeHandle &nh_;
			ros::NodeHandle &planner_nh_;

			fpp_data_classes::RASParams ras_params_;			
	};
}