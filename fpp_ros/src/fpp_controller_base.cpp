#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
	FPPControllerBase::FPPControllerBase(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
										 const fpp_data_classes::FPPControllerParams &fpp_controller_params,
										 ros::NodeHandle &nh,
										 ros::NodeHandle &planner_nh)
		: fpp_params_(fpp_params),
		  nh_(nh),
		  planner_nh_(planner_nh)
	{ 
		this->planner_name_ = fpp_controller_params.path_planner_name;
		this->costmap_ = fpp_controller_params.costmap;
		this->global_frame_ = fpp_controller_params.global_frame;

		this->initTopics();
		this->initServices();
		this->initActions();
		this->initTimers();

		// Create the target formation contour which defines the optimal distances between 
		// a robot and the formation centre and also the robots themselfs.
		this->target_formation_contour_ = geometry_info::FormationContour();
		for(std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info: this->fpp_params_->getRobotInfoList())
		{
			// Create robot contour object with offset from master as position. 
			// This will position the master at 0/0 and everything else relative to the master
			std::shared_ptr<geometry_info::RobotContour> robot_contour =
				std::make_shared<geometry_info::RobotContour>(robot_info->robot_name,
															  robot_info->offset,
															  0.0);

			// Get robot outline through costmap and add corners to robot object
			fpp_msgs::RobotOutline robot_outline_msg;
			robot_outline_msg.request.robot_name = robot_info->robot_name;
			this->get_robot_outline_src_client_.call(robot_outline_msg);

			robot_contour->addContourCornersGeometryCS(
				this->convPolygonToEigenVector(robot_outline_msg.response.outline.polygon));
			bool result = this->target_formation_contour_.addRobotToFormation(robot_contour);
			robot_contour->createContourEdges();
		}
		// Update the formation contour with ne added robots
		this->target_formation_contour_.updateFormationContour();
		// Move CS of formation to formation centre to robot positions can easily be calculated
		this->target_formation_contour_.moveCSToFormationCentre();

		// Set offset vector from formation centre to robot
		this->formation_to_robot_offset_ = this->target_formation_contour_.getRobotPosGeometryCS(
			this->fpp_params_->getCurrentRobotName());

		// Create transformation object
		this->formation_to_robot_trafo_ = plan_transformation::RigidPlanTransformation(this->fpp_params_->getCurrentRobotName(),
																					   this->formation_to_robot_offset_);

		// Create the reconfiguration splines if a reconfiguration was specified
		if (this->fpp_params_->getRekonfigurationStartIndex() == -1 ||
			this->fpp_params_->getRekonfigurationEndIndex() == -1)
		{
			this->x_reconfiguration_spline_ = nullptr;
			this->y_reconfiguration_spline_ = nullptr;
		}
		else
		{
			this->createReconfigurationSplines();
		}
	}

	void FPPControllerBase::initServices() 
	{
		this->get_robot_outline_src_client_ = this->nh_.serviceClient<fpp_msgs::RobotOutline>(
			"/" + this->fpp_params_->getMasterRobotInfo()->robot_namespace + "/move_base_flex/robot_outline");
        this->get_robot_outline_src_client_.waitForExistence();
	}

	void FPPControllerBase::initTopics()
	{
		// Advertise new topic but dont wait for subscribers, as this topic is not init relevant
        this->robot_plan_pub_ = this->nh_.advertise<nav_msgs::Path>("move_base_flex/plan", 10);

		this->robot_plan_meta_data_pub_ =
			this->planner_nh_.advertise<fpp_msgs::GlobalPlanPoseMetaData>("robot_plan_meta_data", 10);
	}

	void FPPControllerBase::initActions() { }

	void FPPControllerBase::initTimers() { }

	void FPPControllerBase::createReconfigurationSplines()
	{
		Eigen::Vector2f reconfiguration_diff;
		reconfiguration_diff = this->fpp_params_->getCurrentRobotInfo()->reconfig_offset -
							   this->fpp_params_->getCurrentRobotInfo()->offset;

		Eigen::Vector2f x_P0;
		x_P0 << 0,0;
		Eigen::Vector2f x_start_tangent;
		x_start_tangent << 1.0, 0;
		Eigen::Vector2f x_end_tangent;
		x_end_tangent << 1.0, 0;
		Eigen::Vector2f x_P3;
		x_P3 << 1.0, reconfiguration_diff[0];
		this->x_reconfiguration_spline_ = std::make_shared<bezier_splines::CubicBezierSplines>(x_P0, x_P3);
		this->x_reconfiguration_spline_->setStartTangent(x_start_tangent);
		this->x_reconfiguration_spline_->setStartTangentMagnitude(1.0);
		this->x_reconfiguration_spline_->setEndTangent(x_end_tangent);
		this->x_reconfiguration_spline_->setEndTangentMagnitude(1.0);
		this->x_reconfiguration_spline_->calcControlPoints();

		Eigen::Vector2f y_P0;
		y_P0 << 0,0;
		Eigen::Vector2f y_start_tangent;
		y_start_tangent << 1.0, 0;
		Eigen::Vector2f y_end_tangent;
		y_end_tangent << 1.0, 0;
		Eigen::Vector2f y_P3;
		y_P3 << 1.0, reconfiguration_diff[1];
		this->y_reconfiguration_spline_ = std::make_shared<bezier_splines::CubicBezierSplines>(y_P0, y_P3);
		this->y_reconfiguration_spline_->setStartTangent(y_start_tangent);
		this->y_reconfiguration_spline_->setStartTangentMagnitude(1.0);
		this->y_reconfiguration_spline_->setEndTangent(y_end_tangent);
		this->y_reconfiguration_spline_->setEndTangentMagnitude(1.0);
		this->y_reconfiguration_spline_->calcControlPoints();
	}

	Eigen::Vector2f FPPControllerBase::calcReconfigurationStep(int reconfiguration_index,
															   int reconfiguration_distance)
	{
		if(reconfiguration_index == 0)
		{
			Eigen::Vector2f reconfig_diff;
			reconfig_diff << 0, 0;
			return reconfig_diff;
		}

		float iterator = float(reconfiguration_index) / float(reconfiguration_distance);

		float x_diff = this->x_reconfiguration_spline_->calcPointOnBezierSpline(iterator)[1] -
					   this->x_reconfiguration_spline_->calcPointOnBezierSpline(iterator - (1.0 / float(reconfiguration_distance)))[1];
		float y_diff = this->y_reconfiguration_spline_->calcPointOnBezierSpline(iterator)[1] -
					   this->y_reconfiguration_spline_->calcPointOnBezierSpline(iterator - (1.0 / float(reconfiguration_distance)))[1];

		Eigen::Vector2f reconfig_diff;
		reconfig_diff << x_diff, y_diff;
		return reconfig_diff;
	}

	void FPPControllerBase::publishPlan(const ros::Publisher &plan_publisher,
										const std::vector<geometry_msgs::PoseStamped> &plan)
	{
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;
        plan_publisher.publish(path_to_publish);
    }

	void FPPControllerBase::publishPlanMetaData(const ros::Publisher &plan_meta_data_publisher,
												const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		for(int pose_counter = 0; pose_counter < plan.size(); pose_counter++)
		{
			fpp_msgs::GlobalPlanPoseMetaData pose_meta_data;
			pose_meta_data.index = pose_counter;

			pose_meta_data.pose = plan[pose_counter].pose;

			pose_meta_data.pose_2D.x = plan[pose_counter].pose.position.x;
			pose_meta_data.pose_2D.y = plan[pose_counter].pose.position.y;
			pose_meta_data.pose_2D.theta = tf::getYaw(plan[pose_counter].pose.orientation);

			plan_meta_data_publisher.publish(pose_meta_data);
		}
	}

	std::vector<geometry_msgs::PoseStamped> FPPControllerBase::transformFormationToRobotPlan(
		std::vector<geometry_msgs::PoseStamped> &formation_plan)
	{
		std::vector<geometry_msgs::PoseStamped> robot_plan;

		int pose_counter = 0;
		int reconfiguration_counter = 0;
		for(geometry_msgs::PoseStamped &formation_pose: formation_plan)
		{
			Eigen::Vector3f eigen_formation_pose = this->convPoseToEigen(formation_pose.pose);

			if (pose_counter >= this->fpp_params_->getRekonfigurationStartIndex() &&
				pose_counter < this->fpp_params_->getRekonfigurationEndIndex() &&
				this->x_reconfiguration_spline_ != nullptr &&
				this->y_reconfiguration_spline_ != nullptr)
			{
				int reconfiguration_index_diff = this->fpp_params_->getRekonfigurationEndIndex() -
												 this->fpp_params_->getRekonfigurationStartIndex();
				
				Eigen::Vector2f reconfiguration_change;
				reconfiguration_change = this->calcReconfigurationStep(reconfiguration_counter,
																	   reconfiguration_index_diff);
				this->formation_to_robot_trafo_.changeRobotOffset(reconfiguration_change);

				reconfiguration_counter = reconfiguration_counter + 1;
			}

			this->formation_to_robot_trafo_.updateFormationState(eigen_formation_pose);

			geometry_msgs::PoseStamped new_target_robot_pose;
			new_target_robot_pose.header.frame_id = this->global_frame_;
			new_target_robot_pose.header.stamp = ros::Time::now();
			new_target_robot_pose.pose = this->convEigenToPose(this->formation_to_robot_trafo_.getTargetState());

			robot_plan.push_back(new_target_robot_pose);
			pose_counter = pose_counter + 1;
		}

		return robot_plan;
	}

	#pragma region Conversion Methods
	std::vector<Eigen::Vector2f> FPPControllerBase::convPolygonToEigenVector(
		geometry_msgs::Polygon polygon)
	{
		std::vector<Eigen::Vector2f> eigen_polygon;

		for(geometry_msgs::Point32 point: polygon.points)
		{
			Eigen::Vector2f eigen_point;
			eigen_point[0] = point.x;
			eigen_point[1] = point.y;
			eigen_polygon.push_back(eigen_point);
		}

		return eigen_polygon;
	}

	geometry_msgs::Pose FPPControllerBase::convEigenToPose(Eigen::Vector3f eigen_pose)
	{
		geometry_msgs::Pose pose;

		pose.position.x = eigen_pose[0];
		pose.position.y = eigen_pose[1];
		pose.position.z = 0.0;
		pose.orientation = tf::createQuaternionMsgFromYaw(eigen_pose[2]);

		return pose;
	}

	Eigen::Vector3f FPPControllerBase::convPoseToEigen(geometry_msgs::Pose pose)
	{
		Eigen::Vector3f eigen_pose;

		eigen_pose[0] = pose.position.x;
		eigen_pose[1] = pose.position.y;
		eigen_pose[2] = tf::getYaw(pose.orientation);

		return eigen_pose;
	}
	#pragma endregion
}