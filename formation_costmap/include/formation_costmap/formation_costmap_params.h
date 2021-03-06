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
		std::vector<FCRobotParams> formation_robot_params;
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
			#pragma endregion

			#pragma ParamReadingHelper
			double getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name);
			std::vector<Eigen::Vector2f> createRobotOutlineFromXMLRPC(XmlRpc::XmlRpcValue footprint_xmlrpc,
																	  const std::string full_param_name);
			#pragma endregion
	};
}