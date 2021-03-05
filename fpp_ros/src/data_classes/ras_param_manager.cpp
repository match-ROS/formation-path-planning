#include <fpp_ros/data_classes/ras_param_manager.h>

namespace fpp_data_classes
{
	RASParamManager::RASParamManager(ros::NodeHandle &nh, ros::NodeHandle &planner_nh)
		: nh_(nh), planner_nh_(planner_nh)
	{ }

	void RASParamManager::getParams(std::string formation_planner_name)
	{
		std::string path_planner_key;
        if (this->planner_nh_.searchParam("formation_path_planner/" + formation_planner_name,
                                          path_planner_key))
        {
			this->planner_nh_.getParam(path_planner_key + "/default_tolerance",
									   this->ras_params_->default_tolerance);
			int neighbor_type;
			this->planner_nh_.getParam(path_planner_key + "/neighbor_type",
									   neighbor_type);
			this->ras_params_->neighbor_type = (fpp_data_classes::NeighborType)neighbor_type;
			this->planner_nh_.getParam(path_planner_key + "/free_cell_threshold",
									   this->ras_params_->free_cell_threshold);
			this->planner_nh_.getParam(path_planner_key + "/start_straight_distance",
									   this->ras_params_->start_straight_distance);
			this->planner_nh_.getParam(path_planner_key + "/end_straight_distance",
									   this->ras_params_->end_straight_distance);
			this->planner_nh_.getParam(path_planner_key + "/control_point_distance",
									   this->ras_params_->control_point_distance);
			this->planner_nh_.getParam(path_planner_key + "/planning_points_per_spline",
									   this->ras_params_->planning_points_per_spline);
			this->planner_nh_.getParam(path_planner_key + "/minimal_curve_radius",
									   this->ras_params_->minimal_curve_radius);

			// Check if calculated minimal radius is bigger than the param
			// Minimal radius is calculated by getting the distance between the robot that is the furthest away from the formation centre
			// float max_calculated_minimal_curve_radius = 0.0;
			// for(std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info: this->fpp_params_->getRobotInfoList())
			// {
			// 	float calculated_minimal_curve_radius = this->target_formation_contour_.getRobotPosGeometryCS(robot_info->robot_name).norm();
			// 	if(max_calculated_minimal_curve_radius < calculated_minimal_curve_radius)
			// 	{
			// 		max_calculated_minimal_curve_radius = calculated_minimal_curve_radius;
			// 	}
			// }
			
			// if(max_calculated_minimal_curve_radius > minimal_curve_radius)
			// {
			// 	minimal_curve_radius = max_calculated_minimal_curve_radius;
			// }
			// this->initial_path_planner_.setMinimalCurveRadius(minimal_curve_radius);
        }
		else
		{
			ROS_ERROR_STREAM("RASParamManager: Path planner for the FormationPathPlanner not found in the config file.");
		}
	}

	std::shared_ptr<fpp_data_classes::RASParams> RASParamManager::getRASParams()
	{
		return this->ras_params_;
	}
}