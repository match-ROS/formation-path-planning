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
	}

	void FPPControllerBase::initActions() { }

	void FPPControllerBase::initTimers() { }

	void FPPControllerBase::publishPlan(const ros::Publisher &plan_publisher, const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;
        plan_publisher.publish(path_to_publish);
    }

	std::vector<Eigen::Vector2f> FPPControllerBase::convPolygonToEigenVector(geometry_msgs::Polygon polygon)
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
}