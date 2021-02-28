#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include <fpc_ros/data_classes/local_planner_robot_info.h>

namespace fpc
{
	class FPCControllerBase
	{
		public:
			#pragma region Constructors
			FPCControllerBase(
				std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list,
				std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);
			#pragma endregion

			#pragma region ControllerInterface
			virtual void initialize(std::string controller_name,
									costmap_2d::Costmap2D *costmap,
									std::string global_frame);

			virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

			virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance);

			bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
			uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
											 const geometry_msgs::TwistStamped &velocity,
											 geometry_msgs::TwistStamped &cmd_vel,
											 std::string &message);

			virtual void publishMetaData();
			#pragma endregion
			
			#pragma region Getter/Setter
			geometry_msgs::Pose getStartPose() { return this->global_plan_.front().pose; }
			geometry_msgs::Pose getGoalPose() { return this->global_plan_.back().pose; }
			#pragma endregion

			
		protected:
			//! NodeHandle from the node that initializes the fpc controller classes
            ros::NodeHandle &nh_;
            //! NodeHandle that is in the namespace of the planner
            ros::NodeHandle &controller_nh_;

			// Information for used controller
            //! Name of the planer that is used to generate the plan for the formation
            std::string controller_name_;
            //! Direct pointer to the costmap to get updates instantly without the usage of topics
            costmap_2d::Costmap2D *costmap_;
            //! Global frame which is used to transform points into map coordinate system
            std::string global_frame_;

			#pragma region ProcessInfo
			//! This is all the information that was read from the config file about each robot
            std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list_;
			//! This points to the object that contains the information about this robot
            std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info_;
			//! This point to the object that represents the master in the formation path planner
			const std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> master_robot_info_;

			std::vector<geometry_msgs::PoseStamped> global_plan_;
			#pragma endregion



			#pragma region Topics/Services/Actions
			ros::Subscriber robot_amcl_pose_subscriber_;
			geometry_msgs::Pose current_robot_pose_;
			ros::Subscriber robot_odom_subscriber_;
			nav_msgs::OdometryConstPtr current_robot_odom_;
			ros::Subscriber robot_ground_truth_subscriber_; // This subscriber will only work in Gazebo where ground truth is published
			nav_msgs::OdometryConstPtr current_robot_ground_truth_;

			ros::Publisher meta_data_publisher_;
			#pragma endregion

			#pragma CallbackMethods
			void getRobotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
			void getRobotOdomCb(const nav_msgs::OdometryConstPtr &msg);

			void getRobotGroundTruthCb(const nav_msgs::OdometryConstPtr &msg);
			
			#pragma endregion

			#pragma region ProtectedHelperMethods
			/**
             * @brief Helper method for intializing all services
             * 
             */
            virtual void initServices();
            /**
             * @brief Helper method for initializing all topics
             * 
             */
            virtual void initTopics();
            /**
             * @brief Helper method for initializing all timers
             * 
             */
            virtual void initTimers();

			std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> getMasterRobotInfo(
				const std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list);
			#pragma endregion
	};
}