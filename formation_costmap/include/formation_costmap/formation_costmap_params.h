#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <XmlRpc.h>

namespace formation_costmap
{
	struct FCRobotParams
	{
		//! Name of the robot for identification
        std::string robot_name;
		//! Definition if this robot is the master of the formation
		bool master;
        //! Namespace of the robot so it is possible to send topics/services/actions to the robot
		std::string robot_namespace;
		//! Definition for the outline of this robot
        std::vector<Eigen::Vector2f> robot_outline;
		//! Topic name where the position of the robot is being published
		std::string robot_pose_topic;
	};

	struct FormationCostmapParams
	{
		//! For every robot there should be an object with the appropriate info inserted
		std::vector<std::shared_ptr<FCRobotParams>> formation_robot_params;

		std::shared_ptr<FCRobotParams> current_robot_info;
		std::shared_ptr<FCRobotParams> master_robot_info;
	};

	class FormationCostmapParamManager
	{
		public:
			FormationCostmapParamManager(ros::NodeHandle &nh, ros::NodeHandle &costmap_nh);

			void getParams(std::string formation_costmap_name);

			#pragma region Getter/Setter
			std::shared_ptr<FormationCostmapParams> getFormationCostmapParams();
			#pragma endregion

		private:
			ros::NodeHandle &nh_;
			ros::NodeHandle &costmap_nh_;

			#pragma region Params
			std::string formation_costmap_name_;

			std::shared_ptr<FormationCostmapParams> formation_costmap_params_;
			#pragma endregion

			#pragma ParamReadingHelper
			double getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name);
			std::vector<Eigen::Vector2f> createRobotOutlineFromXMLRPC(XmlRpc::XmlRpcValue footprint_xmlrpc,
																	  const std::string full_param_name);
			std::shared_ptr<FCRobotParams> getMasterRobotInfo(
				const std::vector<std::shared_ptr<FCRobotParams>> &robot_info_list);
			#pragma endregion
	};
}