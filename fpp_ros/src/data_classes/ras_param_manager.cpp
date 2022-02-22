#include <fpp_ros/data_classes/ras_param_manager.h>

namespace fpp_data_classes
{
	RASParamManager::RASParamManager(ros::NodeHandle &nh, ros::NodeHandle &planner_nh)
		: nh_(nh), planner_nh_(planner_nh)
	{
		this->formation_info_srv_client_ = this->nh_.serviceClient<fpp_msgs::FormationFootprintInfo>("move_base_flex/footprint_info");
		this->formation_info_srv_client_.waitForExistence();

		this->ras_params_ = std::make_shared<RASParams>();
	}

	void RASParamManager::readParams(std::string formation_planner_name)
	{
		std::string path_planner_key;
        if (this->planner_nh_.searchParam("formation_path_planner/" + formation_planner_name,
                                          path_planner_key))
		{
			if(!this->planner_nh_.getParam(path_planner_key + "/default_tolerance",
										   this->ras_params_->default_tolerance))
			{
				this->reportParamError("default_tolerance");
			}
			int neighbor_type;
			if(!this->planner_nh_.getParam(path_planner_key + "/neighbor_type",
										   neighbor_type))
			{
				this->reportParamError("neighbor_type");
			}
			this->ras_params_->neighbor_type = (fpp_data_classes::NeighborType)neighbor_type;
			if(!this->planner_nh_.getParam(path_planner_key + "/free_cell_threshold",
										   this->ras_params_->free_cell_threshold))
			{
				this->reportParamError("free_cell_threshold");
			}
			if(!this->planner_nh_.getParam(path_planner_key + "/start_straight_distance",
									   this->ras_params_->start_straight_distance))
			{
				this->reportParamError("start_straight_distance");
			}
			if(!this->planner_nh_.getParam(path_planner_key + "/end_straight_distance",
									   this->ras_params_->end_straight_distance))
			{
				this->reportParamError("end_straight_distance");
			}
			// if(!this->planner_nh_.getParam(path_planner_key + "/control_point_distance",
			// 						   this->ras_params_->control_point_distance))
			// {
			// 	this->reportParamError("control_point_distance");
			// }
			if(!this->planner_nh_.getParam(path_planner_key + "/planning_points_per_spline",
									   this->ras_params_->planning_points_per_spline))
			{
				this->reportParamError("planning_points_per_spline");
			}
			if(!this->planner_nh_.getParam(path_planner_key + "/minimal_curve_radius",
									   this->ras_params_->minimal_curve_radius))
			{
				this->reportParamError("minimal_curve_radius");
			}
			if(!this->planner_nh_.getParam(path_planner_key + "/max_iterator_step_size",
									   this->ras_params_->max_iterator_step_size))
			{
				this->reportParamError("max_iterator_step_size");
			}
			if(!this->planner_nh_.getParam(path_planner_key + "/max_diff_to_target_length",
									   this->ras_params_->max_diff_to_target_length))
			{
				this->reportParamError("max_diff_to_target_length");
			}
			if(!this->planner_nh_.getParam(path_planner_key + "/target_spline_length",
									   this->ras_params_->target_spline_length))
			{
				this->reportParamError("target_spline_length");
			}

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

		fpp_msgs::FormationFootprintInfo footprint_info;
		if(this->formation_info_srv_client_.call(footprint_info))
		{
			float max_distance = 0.0;
			for(geometry_msgs::Point32 &robot_pose : footprint_info.response.formation_footprint.points)
			{
				float distance = std::sqrt(std::pow(robot_pose.x, 2) + std::pow(robot_pose.y, 2));
				
				if(distance > max_distance)
				{
					max_distance = distance;	
				}
			}

			this->ras_params_->max_robot_to_formation_centre_dist = max_distance;
		}
		else
		{
			ROS_ERROR_STREAM("RASParamManager: Service call for formation radius failed.");
		}
	}

	void RASParamManager::reportParamError(std::string param_name)
	{
		ROS_ERROR_STREAM("The parameter: " << param_name << " was not set or found! This will result in faulty behaviour");
	}

	void RASParamManager::printInfo()
	{
		ROS_INFO_STREAM("RAS Params:");
		ROS_INFO_STREAM("Default Tolerance: " << this->ras_params_->default_tolerance);
		ROS_INFO_STREAM("Neighbor Type: " << this->ras_params_->neighbor_type);
		ROS_INFO_STREAM("Free Cell Threshold: " << this->ras_params_->free_cell_threshold);
		ROS_INFO_STREAM("Start Straight Distance: " << this->ras_params_->start_straight_distance);
		ROS_INFO_STREAM("End Straight Distance: " << this->ras_params_->end_straight_distance);
		ROS_INFO_STREAM("Max Robot to Formation Centre Distance: " << this->ras_params_->max_robot_to_formation_centre_dist);
		ROS_INFO_STREAM("Planning Point Distance: " << this->ras_params_->planning_points_per_spline);
		ROS_INFO_STREAM("Minimal Curve Radius: " << this->ras_params_->minimal_curve_radius);
		ROS_INFO_STREAM("Maximal Iterator Step Size: " << this->ras_params_->max_iterator_step_size);
		ROS_INFO_STREAM("Maximal Difference To Target Length: " << this->ras_params_->max_diff_to_target_length);
		ROS_INFO_STREAM("Target Spline Length: " << this->ras_params_->target_spline_length);
	}

	std::shared_ptr<fpp_data_classes::RASParams> RASParamManager::getRASParams()
	{
		return this->ras_params_;
	}
}