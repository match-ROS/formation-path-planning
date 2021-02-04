#include <fpp_ros/fpp_controller_master.h>

namespace fpp
{
    FPPControllerMaster::FPPControllerMaster(std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list,
                                             std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info,
                                             ros::NodeHandle &nh,
                                             ros::NodeHandle &planner_nh)
        : FPPControllerBase(robot_info_list, robot_info, nh, planner_nh)
    {
        this->initServices();
        this->initTopics();

        // Initialize formation planner with default values. Set lead vector when all robots are added and centroid can be calculated
        this->formation_contour_ = geometry_info::FormationContour(Eigen::Matrix<float, 2, 1>::Zero(), 0.0);
        
        for(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info: this->robot_info_list_)
        {
            Eigen::Vector2f robot_pose;
            float yaw;
            this->getAMCLPose(robot_info->robot_namespace, robot_pose, yaw);
            
            geometry_info::RobotContour robot_contour;
            robot_contour = geometry_info::RobotContour(robot_pose, yaw, robot_info->robot_name);
            
            for(Eigen::Vector2f corner: robot_info->robot_outline)
            {
                robot_contour.addContourCornerGeometryCS(corner);
            }
            robot_contour.createContourEdges();

            this->robot_outline_list_.insert(std::pair<std::string, geometry_info::RobotContour>(robot_info->robot_name, robot_contour));
            this->formation_contour_.addRobotToFormation(robot_contour);
        }
        this->formation_contour_.updateFormationContour();
		this->formation_centre_ = this->formation_contour_.calcCentroidWorldCS();
		this->formation_contour_.moveCoordinateSystem(this->formation_centre_, 0.0);
        
        // Initialize the minimal circle enclosing the formation
        this->formation_enclosing_circle_ = geometry_info::MinimalEnclosingCircle();
        // this->formation_enclosing_circle_.calcMinimalEnclosingCircle(this->formation_centre_,
        //                                                              this->formation_contour_.getCornerPointsWorldCS());
        this->formation_enclosing_circle_.calcMinimalEnclosingCircle(this->formation_contour_.getCornerPointsWorldCS());

        this->callDynamicCostmapReconfigure();

        this->publishFootprint();
        
        this->initTimers();
    }

    void FPPControllerMaster::initServices()
    {
		FPPControllerBase::initServices();
        this->dyn_rec_inflation_srv_client_ = this->nh_.serviceClient<fpp_msgs::DynReconfigure>("/dyn_reconfig_inflation");
        this->dyn_rec_inflation_srv_client_.waitForExistence();
    }

    void FPPControllerMaster::initTopics()
    {
		FPPControllerBase::initTopics();
        this->formation_footprint_pub_ = this->nh_.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10);
        while(this->formation_footprint_pub_.getNumSubscribers() < 1)
            ros::Duration(0.01).sleep();

        // Advertise new topic but dont wait for subscribers, as this topic is not init relevant
        this->formation_plan_pub_ = this->nh_.advertise<nav_msgs::Path>("formation_plan", 10);
    }

    void FPPControllerMaster::initTimers()
    {
		FPPControllerBase::initTimers();
        this->footprint_timer_ = this->nh_.createTimer(ros::Duration(1.0), &FPPControllerMaster::footprintTimerCallback, this);
    }

    void FPPControllerMaster::initialize(std::string planner_name,
                                         costmap_2d::Costmap2D *costmap,
                                         std::string global_frame)
    {
        FPPControllerBase::initialize(planner_name, costmap, global_frame);

        this->initial_path_planner_ = path_planner::SplinedRelaxedAStar(this->planner_name_,
                                                                        this->costmap_,
                                                                        this->global_frame_);
        this->readParams(planner_name);

		ROS_INFO_STREAM("robot_name: " << this->robot_info_->robot_name);

		// Initialize move_base action servers to the slave robots
		for(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info: this->robot_info_list_)
		{
			if(robot_info->robot_name != this->robot_info_->robot_name)
			{
				std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> slave_move_base_as =
					std::make_shared<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>(robot_info->robot_namespace + "/move_base_flex/move_base", true);
				ROS_INFO_STREAM("FPPControllerMaster::initialize: Waiting for " << robot_info->robot_namespace << "/move_base_flex/move_base action server");
				slave_move_base_as->waitForServer();

				this->slave_move_base_as_list_.push_back(slave_move_base_as);
			}
		}
    }

    void FPPControllerMaster::readParams(std::string name)
    {
        std::string path_planner_name_key;
        if(this->planner_nh_.searchParam("used_formation_planner", path_planner_name_key))
        {
            this->planner_nh_.param<std::string>(path_planner_name_key,
                                                 this->used_formation_planner_,
                                                 "SplinedRelaxedAStar");
        }        
        else
        {
            ROS_ERROR_STREAM("FPPControllerMaster: The planner that is used to generate the initial path of the formation must be defined in used_formation_planner");
        }

        std::string path_planner_key;
        if (this->planner_nh_.searchParam("formation_path_planner/" + this->used_formation_planner_,
                                          path_planner_key))
        {
            float default_tolerance;
            this->planner_nh_.getParam(path_planner_key + "/default_tolerance", default_tolerance);
            int neighbor_type;
            this->planner_nh_.getParam(path_planner_key + "/neighbor_type", neighbor_type);
            int free_cell_thresshold;
            this->planner_nh_.getParam(path_planner_key + "/free_cell_thresshold", free_cell_thresshold);

            this->initial_path_planner_.setDefaultTolerance(default_tolerance);
            this->initial_path_planner_.setNeighborType(neighbor_type);
            this->initial_path_planner_.setFreeCellThreshhold(free_cell_thresshold);

            ROS_INFO_STREAM("1: " << default_tolerance << " 2: " << neighbor_type << " 3: " << free_cell_thresshold);
        }
    }

    void FPPControllerMaster::execute(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal,
                                      std::vector<geometry_msgs::PoseStamped> &plan)
    {
        ROS_INFO_STREAM("Start: x: " << start.pose.position.x << " y: " << start.pose.position.y << "\n");
        ROS_INFO_STREAM("Goal: x: " << goal.pose.position.x << " y: " << goal.pose.position.y << "\n");
        
        geometry_msgs::PoseStamped formation_start = start;
		std::vector<geometry_msgs::PoseStamped> formation_plan;
        formation_start.pose.position.x = this->formation_centre_[0];
        formation_start.pose.position.y = this->formation_centre_[1];
        ROS_INFO_STREAM("Formation Start: x: " << formation_start.pose.position.x << " y: " << formation_start.pose.position.y << "\n");
        this->initial_path_planner_.makePlan(formation_start, goal, formation_plan);

		// Eigen::Vector2f robot_offset = -this->formation_centre_;
		Eigen::Vector2f robot_offset = this->formation_contour_.getRobotPosGeometryCS(this->robot_info_->robot_name);
		ROS_INFO_STREAM("robot_offset: " << robot_offset);

		this->calcOffsetPlan(formation_plan, plan, robot_offset);
		ROS_INFO_STREAM("formation:" << formation_plan[0].pose.position.x << " " << formation_plan[0].pose.position.y);
		ROS_INFO_STREAM("plan:" << plan[0].pose.position.x << " " << plan[0].pose.position.y);
		// Call move_base action servers of each slave robot to initialize the global planning of their path
		for(std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> &slave_move_base_as: this->slave_move_base_as_list_)
		{
			mbf_msgs::MoveBaseGoal msg;
			msg.controller = "dwa";
			msg.planner = "FormationPathPlanner";
			msg.recovery_behaviors.push_back("rotate_recovery");
			msg.recovery_behaviors.push_back("clear_costmap");
			msg.recovery_behaviors.push_back("moveback_recovery");
			msg.target_pose = goal;
			slave_move_base_as->sendGoal(msg);
		}

        this->publishPlan(this->formation_plan_pub_, formation_plan);
		this->publishPlan(this->robot_plan_pub_, plan);
    }


    geometry_msgs::PoseWithCovarianceStampedConstPtr FPPControllerMaster::getAMCLPose(std::string robot_namespace)
    {
        std::string amcl_pose_topic = robot_namespace + "/amcl_pose";
        geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose_ptr;
        robot_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(amcl_pose_topic);
        return robot_pose_ptr;
    }

    void FPPControllerMaster::getAMCLPose(std::string robot_namespace, Eigen::Vector2f &robot_pose, float &yaw)
    {
        geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose_ptr;
        robot_pose_ptr = this->getAMCLPose(robot_namespace);

        if(robot_pose_ptr == nullptr)
        {
            ROS_ERROR_STREAM("FormationPathPlanner: No message was received from the amcl_pose topic in the robot_namespace: " << robot_namespace);
            robot_pose << 0, 0;
            yaw = 0;
        }
        else
        {
            robot_pose << robot_pose_ptr->pose.pose.position.x, robot_pose_ptr->pose.pose.position.y;
            yaw = tf::getYaw(robot_pose_ptr->pose.pose.orientation);
        }
    }

    void FPPControllerMaster::updateFootprint()
    {
        for(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info: this->robot_info_list_)
        {
            Eigen::Vector2f new_robot_pose;
            float new_rotation;
            this->getAMCLPose(robot_info->robot_namespace, new_robot_pose, new_rotation);

            this->formation_contour_.updateRobotPose(robot_info->robot_name, new_robot_pose, new_rotation);
        }
        this->formation_contour_.updateFormationContour();
    }

    void FPPControllerMaster::publishFootprint()
    {
        geometry_msgs::PolygonStamped formation_footprint_msg;
        formation_footprint_msg.header.frame_id = "map";
        formation_footprint_msg.header.stamp = ros::Time::now();

        std::vector<Eigen::Vector2f> formation_corner_points = this->formation_contour_.getCornerPointsWorldCS();
        for(Eigen::Vector2f corner: formation_corner_points)
        {
            geometry_msgs::Point32 corner_point;
            corner_point.x = corner[0];
            corner_point.y = corner[1];
            corner_point.z = 0.0;
            formation_footprint_msg.polygon.points.push_back(corner_point);
        }

        this->formation_footprint_pub_.publish(formation_footprint_msg);
    }

    void FPPControllerMaster::callDynamicCostmapReconfigure()
    {
        fpp_msgs::DynReconfigure dyn_reconfig_msg;
        dyn_reconfig_msg.request.new_inflation_radius = this->formation_enclosing_circle_.getCircleRadius();
        dyn_reconfig_msg.request.robot_namespace = this->robot_info_->robot_namespace;
        ros::Duration(0.1).sleep();
        this->dyn_rec_inflation_srv_client_.call(dyn_reconfig_msg);
    }

    void FPPControllerMaster::footprintTimerCallback(const ros::TimerEvent& e)
    {
        this->updateFootprint();
        this->publishFootprint();        
    }
}