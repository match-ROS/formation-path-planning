#include <fpc_ros/fpc_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fpc::FormationPathController, mbf_costmap_core::CostmapController)
PLUGINLIB_EXPORT_CLASS(fpc::FormationPathController, nav_core::BaseLocalPlanner)

namespace fpc
{
	FormationPathController::FormationPathController()
		: initialized_(false)
	{
		ROS_ERROR_STREAM("Constructor");
	}

	void FormationPathController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		if(!this->initialized_)
		{
			ROS_ERROR_STREAM("initialize");

			// Store all process info
			this->controller_name_ = name;
			this->tf_buffer_ = tf;
			this->costmap_ros_ = costmap_ros;
			this->costmap_ = this->costmap_ros_->getCostmap();
			this->global_frame_ = this->costmap_ros_->getGlobalFrameID();

			this->nh_ = ros::NodeHandle();
			this->planner_nh_ = ros::NodeHandle("~/" + this->controller_name_);
		
			this->tf_prefix_ = tf::getPrefixParam(this->nh_);
			this->robot_ns_ = this->nh_.getNamespace();

			// Read and store all params that are set by the yaml config
			this->getParams();

			// CREATE MASTER SLAVE OBJECTS HERE

			this->initialized_ = true;
			ROS_INFO_STREAM("FormationPathController::initialize: FPC has been succesfully initialized");
		}
		else
		{
			ROS_WARN_STREAM("FormationPathController::initialize: FPC has already been initialized");
		}
	}

	bool FormationPathController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		ROS_ERROR_STREAM("setPlan");
		// Save the complete global plan
		this->global_plan_ = std::make_shared<std::vector<geometry_msgs::PoseStamped>>(plan);

		return true;
	}

	bool FormationPathController::isGoalReached()
	{
		return this->isGoalReached(this->xy_default_tolerance_, this->yaw_default_tolerance_);
	}

	bool FormationPathController::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		// Glaube ich muss hier auch etwas übergreifendes implementieren, falls der Master meldet Ziel ist erreicht, dann müssen die anderen ROboter das auch melden
		// Außer ich will vllt noch versuchen den restlichen Regelfehler auszugleichen, aber da sollte ja keiner sein

		ROS_ERROR_STREAM("isGoalReached2");
		return false;
	}

	bool FormationPathController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		ROS_ERROR_STREAM("computeVelocityCommands1");
	}

	uint32_t FormationPathController::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
															  const geometry_msgs::TwistStamped &velocity,
															  geometry_msgs::TwistStamped &cmd_vel,
															  std::string &message)
	{
		ROS_ERROR_STREAM("computeVelocityCommands2");

		return 0;
	}

	bool FormationPathController::cancel()
	{
		ROS_ERROR_STREAM("cancel");
	}


	geometry_msgs::PoseStamped FormationPathController::getGlobalStartPose()
	{
		return this->global_plan_->front();
	}

	geometry_msgs::PoseStamped FormationPathController::getGlobalGoalPose()
	{
		return this->global_plan_->back();
	}

	//////////////////////////////////////////
	// Private methods
	//////////////////////////////////////////
	void FormationPathController::getParams()
	{
		// Get robot name of the current robot
        std::string robot_name_key;
        if(this->nh_.searchParam("robot_name", robot_name_key))
        {
            this->nh_.getParam(robot_name_key, this->robot_name_);
        }
        else
        {
            ROS_ERROR("No RobotName found in the robot namespace. This param \"robot_name\" has to be set.");
        }

		// Get default tolerances for the planner
        this->planner_nh_.param<double>("xy_default_tolerance", this->xy_default_tolerance_, 0.1);
		this->planner_nh_.param<double>("yaw_default_tolerance", this->yaw_default_tolerance_, 0.1);
	
		// Get information about the robots that are participating in the formation
		XmlRpc::XmlRpcValue formation_config;
        this->planner_nh_.getParam("formation_config", formation_config);
        if(formation_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
            for(robot_iterator = formation_config.begin(); robot_iterator != formation_config.end(); robot_iterator++)
            {
				std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> robot_info =
					std::make_shared<fpc_data_classes::LocalPlannerRobotInfo>();

                robot_info->robot_name = robot_iterator->first;

                XmlRpc::XmlRpcValue robot_info_xmlrpc = robot_iterator->second;

                if(robot_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    if(robot_info_xmlrpc.hasMember("master") && robot_info_xmlrpc["master"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        robot_info->fpc_master = (bool)robot_info_xmlrpc["master"];
                    }
                    else
                    {
						// master parameter is not available for the robot so robot is not master 
                        robot_info->fpc_master = false;
                    }

                    if(robot_info_xmlrpc.hasMember("namespace") && robot_info_xmlrpc["namespace"].getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        robot_info->robot_namespace = static_cast<std::string>(robot_info_xmlrpc["namespace"]);
                    }
                }

                this->robot_info_list_.push_back(robot_info);
                if(robot_info->robot_name == this->robot_name_)
				{
					this->current_robot_info_ = robot_info;
				}
            }
        }
	}
}