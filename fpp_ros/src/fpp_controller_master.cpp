#include <fpp_ros/fpp_controller_master.h>

namespace fpp
{
    FPPControllerMaster::FPPControllerMaster(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
                                             ros::NodeHandle &nh,
                                             ros::NodeHandle &planner_nh)
        : FPPControllerBase(fpp_params, nh, planner_nh)
    {
        this->initServices();
        this->initTopics();
		this->initActions();

        // Initialize the target formation contour that takes amcl from master robot and then the wanted offset for the other robots
        // this->target_formation_contour_ = footprint_generation::FormationFootprintRos(Eigen::Matrix<float, 2, 1>::Zero(), 0.0);
        
		// Add master robot by amcl pose
		// Eigen::Vector2f master_robot_pose;
		// float master_yaw;
		// this->getAMCLPose(this->fpp_params_->getCurrentRobotNamespace(), master_robot_pose, master_yaw);
		// footprint_generation::RobotFootprintRos master_robot_contour = footprint_generation::RobotFootprintRos(
		// 	this->nh_, 
		// 	this->fpp_params_->getCurrentRobotName(), 
		// 	this->fpp_params_->getCurrentRobotNamespace(),
		// 	this->fpp_params_->getRobotInfoByRobotName(this->fpp_params_->getCurrentRobotName())->robot_pose_topic_name);
		// this->robot_outline_list_.insert(std::pair<std::string, footprint_generation::RobotFootprintRos>(
		// 	this->fpp_params_->getCurrentRobotName(), master_robot_contour));
		// this->target_formation_contour_.addRobotToFormation(master_robot_contour);

		// Add slave robots by relative offset to master robot. Take same yaw angle as this should be equal
        // for(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info_it: this->fpp_params_->getRobotInfoList())
        // {
		// 	if(robot_info_it->robot_name != this->fpp_params_->getCurrentRobotName())
		// 	{
		// 		footprint_generation::RobotFootprintRos slave_robot_contour = footprint_generation::RobotFootprintRos(
		// 			this->nh_, 
		// 			robot_info_it->robot_name, 
		// 			robot_info_it->robot_namespace,
		// 			robot_info_it->robot_pose_topic_name);

		// 		slave_robot_contour.addContourCornersGeometryCS(robot_info_it->robot_outline);
		// 		slave_robot_contour.createContourEdges();

		// 		this->robot_outline_list_.insert(std::pair<std::string, footprint_generation::RobotFootprintRos>(
		// 			robot_info_it->robot_name, slave_robot_contour));
		// 		this->target_formation_contour_.addRobotToFormation(slave_robot_contour);
		// 	}
        // }
        // this->target_formation_contour_.updateFormationContour();
		// this->formation_centre_ = this->target_formation_contour_.calcCentroidWorldCS();
		// this->target_formation_contour_.moveCoordinateSystem(this->formation_centre_, 0.0);
        
        // Initialize the minimal circle enclosing the formation
        // this->formation_enclosing_circle_ = geometry_info::MinimalEnclosingCircle();
		// For now the minimal enclosing circle will be bigger than the smallest circle possible
        // this->formation_enclosing_circle_.calcMinimalEnclosingCircle(this->formation_centre_,
        //                                                              this->target_formation_contour_.getCornerPointsWorldCS());
        // this->formation_enclosing_circle_.calcMinimalEnclosingCircle(this->target_formation_contour_.getCornerPointsWorldCS());

        this->callDynamicCostmapReconfigure();

        this->initTimers();
    }

	void FPPControllerMaster::initialize(std::string planner_name,
                                         costmap_2d::Costmap2D *costmap,
                                         std::string global_frame)
    {
        FPPControllerBase::initialize(planner_name, costmap, global_frame);

		this->readParams(planner_name);
		
		// Initialize real footprint. This should be put into an own costmap layer later.
		// this->real_formation_contour_ = this->createFootprintObj(this->fpp_params_->getRobotInfoList());

		this->initial_path_planner_ = path_planner::SplinedRelaxedAStar(this->planner_name_,
																		this->costmap_,
																		this->global_frame_,
																		this->ras_param_manager_->getRASParams());
		
		ROS_INFO_STREAM("robot_name: " << this->fpp_params_->getCurrentRobotName());
    }

	void FPPControllerMaster::execute(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal,
                                      std::vector<geometry_msgs::PoseStamped> &plan)
    {
        ROS_INFO_STREAM("Start: x: " << start.pose.position.x << " y: " << start.pose.position.y);
        ROS_INFO_STREAM("Goal: x: " << goal.pose.position.x << " y: " << goal.pose.position.y);
        
        geometry_msgs::PoseStamped formation_start = start;
		std::vector<geometry_msgs::PoseStamped> formation_plan;
        formation_start.pose.position.x = this->formation_centre_[0];
        formation_start.pose.position.y = this->formation_centre_[1];
        ROS_INFO_STREAM("Formation Start: x: " << formation_start.pose.position.x << " y: " << formation_start.pose.position.y);
        this->initial_path_planner_.makePlan(formation_start, goal, formation_plan);

		this->calcRobotPlans(formation_plan);
		plan = this->robot_plan_list_[this->fpp_params_->getCurrentRobotName()];
		
		// Call move_base action servers of each slave robot to initialize the global planning of their path
		for(std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> &slave_move_base_as: this->slave_move_base_as_list_)
		{
			mbf_msgs::MoveBaseGoal msg;
			msg.controller = "FormationPathController";
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
			ROS_ERROR_STREAM("FPPControllerMaster: The planner that is used to generate the initial path "
							 "of the formation must be defined in used_formation_planner");
		}


    }

    void FPPControllerMaster::initServices()
    {
		FPPControllerBase::initServices();
        this->dyn_rec_inflation_srv_client_ = this->nh_.serviceClient<fpp_msgs::DynReconfigure>("/dyn_reconfig_inflation");
        this->dyn_rec_inflation_srv_client_.waitForExistence();

		this->get_robot_plan_srv_server_ = this->nh_.advertiseService("get_robot_plan",
																	  &FPPControllerMaster::getRobotPlanCb,
																	  this);
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

	void FPPControllerMaster::initActions()
	{
		// Initialize move_base action servers to the slave robots
		for(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info: this->fpp_params_->getRobotInfoList())
		{
			if(robot_info->robot_name != this->fpp_params_->getCurrentRobotName())
			{
				std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> slave_move_base_as =
					std::make_shared<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>(
						robot_info->robot_namespace + "/move_base_flex/move_base", true);
				ROS_INFO_STREAM("FPPControllerMaster::initialize: Waiting for "
								<< robot_info->robot_namespace << "/move_base_flex/move_base action server");
				slave_move_base_as->waitForServer();

				this->slave_move_base_as_list_.push_back(slave_move_base_as);
			}
		}
	}

    void FPPControllerMaster::initTimers()
    {
		FPPControllerBase::initTimers();
		// this->footprint_timer_ = this->nh_.createTimer(ros::Duration(0.1),
		// 											   &FPPControllerMaster::footprintTimerCallback,
		// 											   this);
	}

	// std::shared_ptr<footprint_generation::FormationFootprintRos> FPPControllerMaster::createFootprintObj(
	// 	std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> robot_info_list)
	// {
	// 	std::shared_ptr<footprint_generation::FormationFootprintRos> formation_footprint =
	// 		std::make_shared<footprint_generation::FormationFootprintRos>();
	// 	for(const std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info_it: robot_info_list)
    //     {
	// 		ROS_INFO_STREAM(robot_info_it->robot_name);
	// 		std::shared_ptr<footprint_generation::RobotFootprintRos> robot_contour =
	// 			std::make_shared<footprint_generation::RobotFootprintRos>(this->nh_,
	// 																	  robot_info_it->robot_name,
	// 																	  robot_info_it->robot_namespace,
	// 																	  robot_info_it->robot_pose_topic_name);

	// 		for(Eigen::Vector2f corner: robot_info_it->robot_outline)
	// 		{
	// 			robot_contour->addContourCornerGeometryCS(corner);
	// 		}
	// 		robot_contour->createContourEdges();
	// 		formation_footprint->addRobotToFormation(robot_contour);
    //     }

	// 	return formation_footprint;
	// }

	void FPPControllerMaster::calcRobotPlans(const std::vector<geometry_msgs::PoseStamped> &formation_plan)
	{
		// Clear existing plans and create new empty lists for the plans for each robot
		this->robot_plan_list_.clear();
		// footprint_generation::FormationFootprintRos path_planner_formation = this->target_formation_contour_;
		// for(const std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info_it: this->fpp_params_->getRobotInfoList())
		// {
		// 	this->robot_plan_list_.insert(std::pair<std::string, std::vector<geometry_msgs::PoseStamped>>(robot_info_it->robot_name,
		// 																								  std::vector<geometry_msgs::PoseStamped>()));
		// }
		// // Push FormationContour along the path and save each robot position in the plan list
		// for(geometry_msgs::PoseStamped formation_pose_in_plan: formation_plan)
		// {
		// 	// ROS_INFO_STREAM("formation pose: " << formation_pose_in_plan.pose.position.x << " " << formation_pose_in_plan.pose.position.y << " " << tf::getYaw(formation_pose_in_plan.pose.orientation));
		// 	Eigen::Vector2f new_lead_vector_world_cs;
		// 	new_lead_vector_world_cs << formation_pose_in_plan.pose.position.x,
		// 		formation_pose_in_plan.pose.position.y;
		// 	float new_rotation = tf::getYaw(formation_pose_in_plan.pose.orientation);
		// 	path_planner_formation.moveContour(new_lead_vector_world_cs, new_rotation);

		// 	for(const std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info_it: this->fpp_params_->getRobotInfoList())
		// 	{
		// 		geometry_msgs::PoseStamped new_pose;
		// 		new_pose.header.stamp = formation_pose_in_plan.header.stamp;
		// 		new_pose.header.frame_id = formation_pose_in_plan.header.frame_id;

		// 		Eigen::Vector2f pose = path_planner_formation.getRobotPosWorldCS(robot_info_it->robot_name);
		// 		new_pose.pose.position.x = pose[0];
		// 		new_pose.pose.position.y = pose[1];
		// 		new_pose.pose.position.z = 0.0;

		// 		// new_pose.pose.orientation = formation_pose_in_plan.pose.orientation;

		// 		this->robot_plan_list_[robot_info_it->robot_name].push_back(new_pose);
		// 	}
		// }

		// //Calc orientation for each point of the robot plans
		// for(const std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info_it: this->fpp_params_->getRobotInfoList())
		// {
		// 	// for(geometry_msgs::PoseStamped formation_pose_in_plan_: this->robot_plan_list_[robot_info_it->robot_name])
		// 	// {
				
		// 	// }

		// 	// < is necessary because we just copy elements from one vector (0 until size()) to the other
        //     for(uint path_counter = 0; path_counter < this->robot_plan_list_[robot_info_it->robot_name].size(); path_counter++)
        //     {
        //         // Calculate orientation for each point of the plan with the current position and the last one
        //         if(path_counter == 0) // No previous point so orientation of start will be taken
        //         {
        //             this->robot_plan_list_[robot_info_it->robot_name][path_counter].pose.orientation = formation_plan[0].pose.orientation;
        //         }
        //         else // Some other points are before, so orientation can be calculated
        //         {
		// 			float delta_x = this->robot_plan_list_[robot_info_it->robot_name][path_counter].pose.position.x -
		// 							this->robot_plan_list_[robot_info_it->robot_name][path_counter - 1].pose.position.x;
		// 			float delta_y = this->robot_plan_list_[robot_info_it->robot_name][path_counter].pose.position.y -
		// 							this->robot_plan_list_[robot_info_it->robot_name][path_counter - 1].pose.position.y;
        //             double yaw_angle = std::atan2(delta_y, delta_x);
        //             this->robot_plan_list_[robot_info_it->robot_name][path_counter].pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);
        //         }

        //         // last_pose = pose; // Safe pose for next iteration
        //         // plan.insert(plan.begin() + plan.size(), pose);
        //     }
		// }
	}

    void FPPControllerMaster::callDynamicCostmapReconfigure()
    {
        fpp_msgs::DynReconfigure dyn_reconfig_msg;
        dyn_reconfig_msg.request.new_inflation_radius = this->formation_enclosing_circle_.getCircleRadius();
        dyn_reconfig_msg.request.robot_namespace = this->fpp_params_->getCurrentRobotNamespace();
        ros::Duration(0.1).sleep();
        this->dyn_rec_inflation_srv_client_.call(dyn_reconfig_msg);
    }

    // void FPPControllerMaster::footprintTimerCallback(const ros::TimerEvent& e)
    // {
    //     this->formation_footprint_pub_.publish(this->real_formation_contour_->getFormationFootprint());
    // }

	bool FPPControllerMaster::getRobotPlanCb(fpp_msgs::GetRobotPlan::Request &req, fpp_msgs::GetRobotPlan::Response &res)
	{
		nav_msgs::Path plan;
		plan.header.frame_id = this->robot_plan_list_[req.robot_name].front().header.frame_id;
		plan.header.stamp = this->robot_plan_list_[req.robot_name].front().header.stamp;
		plan.poses = this->robot_plan_list_[req.robot_name];
		res.robot_plan = plan;
	}
}