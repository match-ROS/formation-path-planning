#include <fpp_ros/fpp_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fpp::FormationPathPlanner, mbf_costmap_core::CostmapPlanner)
PLUGINLIB_EXPORT_CLASS(fpp::FormationPathPlanner, nav_core::BaseGlobalPlanner)

namespace fpp
{
    FormationPathPlanner::FormationPathPlanner() : 
        initialized_(false)
    { }

	FormationPathPlanner::FormationPathPlanner(std::string name,
											   costmap_2d::Costmap2DROS *costmap_ros)
		: initialized_(false)
	{
		if(!this->initialized_)
		{
			this->initialize(name, costmap_ros);
		}
    }

	void FormationPathPlanner::initialize(std::string name,
										  costmap_2d::Costmap2DROS *costmap_ros)
	{
        if(!this->initialized_)
        {
            this->initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        }
    }

	void FormationPathPlanner::initialize(std::string name,
										  costmap_2d::Costmap2D *costmap,
										  std::string global_frame)
	{
        if(!this->initialized_)
        {
            // Safe parameter for planning
            this->path_planner_name_ = name;

            // Initialize node handle that points to namespace of planner
            this->nh_ = ros::NodeHandle();
            this->planner_nh_ = ros::NodeHandle("~/" + this->path_planner_name_);

            // Get all params from the config file for the global path planner
			this->fpp_params_ = std::make_shared<fpp_data_classes::FPPParamManager>(this->nh_, this->planner_nh_);
			this->fpp_params_->getParams();

			ROS_INFO_STREAM("Initializing Formation Path Planner in namespace: "
							<< this->fpp_params_->getCurrentRobotName().c_str());

			// Initialize the controller object that defines if this is master or slave.
			if(this->fpp_params_->isCurrentRobotMaster())
            {
                this->fpp_controller_ = std::make_shared<FPPControllerMaster>(this->fpp_params_,
                                                                              this->nh_,
                                                                              this->planner_nh_);
            }
            else
            {
                this->fpp_controller_ = std::make_shared<FPPControllerSlave>(this->fpp_params_,
                                                                             this->nh_,
                                                                             this->planner_nh_);
            }
            this->fpp_controller_->initialize(name, costmap, global_frame);

            initialized_ = true; // Initialized method was called so planner is now initialized

            ROS_INFO_STREAM("Formation Path Planner finished intitialization.");
        }
        else
        {
            ROS_WARN_STREAM("Formation Path Planner has already been initialized");
        }
    }

	bool FormationPathPlanner::makePlan(const geometry_msgs::PoseStamped &start,
										const geometry_msgs::PoseStamped &goal,
										std::vector<geometry_msgs::PoseStamped> &plan)
	{
		double cost;
        std::string message;
        return 10 > this->makePlan(start, goal, 0.1, plan, cost, message);
	}

	uint32_t FormationPathPlanner::makePlan(const geometry_msgs::PoseStamped &start,
											const geometry_msgs::PoseStamped &goal,
											double tolerance,
											std::vector<geometry_msgs::PoseStamped> &plan,
											double &cost,
											std::string &message)
	{
        if(!initialized_) //Planner was not initialized. Abort
        {
			ROS_ERROR_STREAM("RelaxedAStar planner was not initialized yet. "
							 "Please initialize the planner before usage.");
			return mbf_msgs::GetPathResult::NOT_INITIALIZED;
		}

        this->fpp_controller_->execute(start, goal, plan);
        
        return 0;
    }

    bool FormationPathPlanner::cancel()
    {
        ROS_INFO_STREAM("FormationPathPlanner: CANCEL");
        return false; 
    }
}