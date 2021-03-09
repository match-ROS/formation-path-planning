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
	}

    // void FPPControllerBase::initialize(std::string planner_name,
    //                                    costmap_2d::Costmap2D *costmap,
    //                                    std::string global_frame)
    // {
    //     this->planner_name_ = planner_name;
    //     this->costmap_ = costmap;
    //     this->global_frame_ = global_frame;
    // }

	void FPPControllerBase::initServices() { }

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
}