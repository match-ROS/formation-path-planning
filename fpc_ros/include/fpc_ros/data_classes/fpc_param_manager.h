#pragma once

#include "ros/ros.h"

#include <string>
#include <memory>
#include <fpc_ros/data_classes/fpc_param_info.hpp>

namespace fpc_data_classes
{
	class FPCParamManager
	{
		public:
			FPCParamManager(ros::NodeHandle &nh, ros::NodeHandle &controller_nh);

			void getParams();

			std::shared_ptr<FPCParamInfo> getLocalPlannerInfo();

		private:
			ros::NodeHandle &nh_;
			ros::NodeHandle &controller_nh_;

			std::shared_ptr<FPCParamInfo> fpc_param_info_;

			double getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name);
	};
}