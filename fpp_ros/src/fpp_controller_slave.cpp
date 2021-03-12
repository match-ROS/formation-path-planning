#include <fpp_ros/fpp_controller_slave.h>

namespace fpp
{
	FPPControllerSlave::FPPControllerSlave(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
										   const fpp_data_classes::FPPControllerParams &fpp_controller_params,
										   ros::NodeHandle &nh,
										   ros::NodeHandle &planner_nh)
		: FPPControllerBase(fpp_params, fpp_controller_params, nh, planner_nh)
	{
		this->initTopics();	
    }

    void FPPControllerSlave::execute(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     std::vector<geometry_msgs::PoseStamped> &plan)
    {
		// fpp_msgs::GetRobotPlan robot_plan_msg;
		// robot_plan_msg.request.robot_name = this->fpp_params_->getCurrentRobotName();
		// this->get_robot_plan_srv_client_.call(robot_plan_msg);

		// plan = robot_plan_msg.response.robot_plan.poses;

		// this->publishPlan(this->robot_plan_pub_, plan);

		while(this->formation_plan_ == nullptr)
		{
			ros::Duration(0.01).sleep();
		}

		plan = this->transformFormationToRobotPlan(this->formation_plan_->poses);
		this->publishPlan(this->robot_plan_pub_, plan);
		this->publishPlanMetaData(this->robot_plan_meta_data_pub_, plan);
    }

	void FPPControllerSlave::initTopics()
	{
		std::string formation_plan_topic;
		formation_plan_topic = "/" + this->fpp_params_->getMasterRobotInfo()->robot_namespace + "/formation_plan";
		this->formation_plan_sub_ = this->nh_.subscribe(formation_plan_topic,
														10,
														&FPPControllerSlave::getFormationPlanCb,
														this);
	}

	void FPPControllerSlave::getFormationPlanCb(nav_msgs::Path formation_plan)
	{
		this->formation_plan_ = std::make_shared<nav_msgs::Path>(formation_plan);
	}
}