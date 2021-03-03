#pragma once

#include "ros/ros.h"

#include <string>
#include <memory>
#include <Eigen/Dense>
#include <XmlRpc.h>

#include <fpp_ros/data_classes/robot_info.h>

namespace fpp_data_classes
{
	class FPPParamManager
	{
		public:
			FPPParamManager(ros::NodeHandle &nh, ros::NodeHandle &planner_nh);

			/**
             * @brief Helper method for reading all parameters that are searched for and specified in the planner_nh namespace
             * 
             */
            void getParams();

			#pragma region Getter/Setter
			std::string getCurrentRobotName();
			std::string getCurrentRobotNamespace();
			float getDefaultTolerance();
			std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> getRobotInfoList();
			std::shared_ptr<fpp_data_classes::RobotInfo> getCurrentRobotInfo();
			std::shared_ptr<fpp_data_classes::RobotInfo> getMasterRobotInfo();
			std::shared_ptr<fpp_data_classes::RobotInfo> getRobotInfoByRobotName(std::string robot_name);
			#pragma endregion

			#pragma region HelperMethods
			bool isCurrentRobotMaster();
			#pragma endregion

		private:
			ros::NodeHandle &nh_;
			ros::NodeHandle &planner_nh_;

			#pragma region Params
			//! The default tolerance that is used if the tolerance of the received goal is not valid
            float default_tolerance_;
			//! Contains all positions of every robot that is part of the formation
            std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> robot_info_list_;
            //! This is a pointer to the RobotInfo object in the robot_info_list for easier access
            std::shared_ptr<fpp_data_classes::RobotInfo> current_robot_info_;
			//! This point to the object in the robot_info_list that represents the master in the formation path planner
			std::shared_ptr<fpp_data_classes::RobotInfo> master_robot_info_;
			#pragma endregion

			#pragma region ParamReadingHelper
			std::shared_ptr<fpp_data_classes::RobotInfo> getMasterRobotInfo(
				const std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list);

			double getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name);
            std::vector<Eigen::Vector2f> createRobotOutlineFromXMLRPC(XmlRpc::XmlRpcValue footprint_xmlrpc,
                                                                      const std::string full_param_name);
			#pragma endregion																	  
	};
}