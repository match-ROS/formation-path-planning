#pragma once

#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <fpc_ros/data_classes/fpc_param_info.hpp>
#include <fpp_msgs/LocalPlannerMetaData.h>

namespace fpc
{
	class FPCControllerBase
	{
		public:
			#pragma region Constructors
			FPCControllerBase(
				std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
				ros::NodeHandle &nh,
				ros::NodeHandle &controller_nh);
			#pragma endregion

			#pragma region ControllerInterface
			virtual void initialize(std::string controller_name,
									tf2_ros::Buffer *tf,
									costmap_2d::Costmap2DROS *costmap_ros);

			virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

			virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance);

			virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
			virtual uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
													 const geometry_msgs::TwistStamped &velocity,
													 geometry_msgs::TwistStamped &cmd_vel,
													 std::string &message);

			virtual void publishMetaData(geometry_msgs::Pose2D target_pose);
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
			costmap_2d::Costmap2DROS *costmap_ros_;
            //! Global frame which is used to transform points into map coordinate system
            std::string global_frame_;
			
			tf2_ros::Buffer *tf_buffer_;

			std::string tf_prefix_;
			std::string robot_ns_;

			#pragma region ProcessInfo
			std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info_;
			// //! This is all the information that was read from the config file about each robot
            // std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list_;
			// //! This points to the object that contains the information about this robot
            // std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info_;
			// //! This point to the object that represents the master in the formation path planner
			// const std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> master_robot_info_;

			geometry_msgs::Twist last_published_cmd_vel_;

			std::vector<geometry_msgs::PoseStamped> global_plan_;

			int pose_index_;
			bool controller_finished_;

			geometry_msgs::PoseStamped last_target_pose_;
			#pragma endregion

			#pragma region Topics/Services/Actions/Timers
			ros::Subscriber robot_amcl_pose_subscriber_;
			geometry_msgs::Pose current_robot_amcl_pose_;
			ros::Subscriber robot_odom_subscriber_;
			nav_msgs::OdometryConstPtr current_robot_odom_;
			ros::Subscriber robot_ground_truth_subscriber_; // This subscriber will only work in Gazebo where ground truth is published
			nav_msgs::OdometryConstPtr current_robot_ground_truth_;

			ros::Publisher cmd_vel_publisher_;
			ros::Publisher meta_data_publisher_;
			fpp_msgs::LocalPlannerMetaData meta_data_msg_; // This is the msg that will be published. Every info can be stored here and will be reset when message is published. 

			ros::Timer controller_timer_;
			#pragma endregion

			#pragma region CallbackMethods
			void getRobotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
			void getRobotOdomCb(const nav_msgs::OdometryConstPtr &msg);

			void getRobotGroundTruthCb(const nav_msgs::OdometryConstPtr &msg);
			
			void onControllerTimerCB(const ros::TimerEvent& timer_event_info);
			#pragma endregion

			#pragma region Controller Methods
			virtual void runController();
			float calcLinVelocity(geometry_msgs::Pose2D diff_vector, float scale_factor);
			float calcRotVelocity(geometry_msgs::Pose2D diff_vector);

			int locateRobotOnPath(geometry_msgs::Pose current_pose);
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

			geometry_msgs::Pose2D convertPose(geometry_msgs::Pose pose_to_convert);
			geometry_msgs::Pose2D calcDiff(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose);
			geometry_msgs::Pose2D calcDiff(geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D end_pose);
			geometry_msgs::Twist calcDiff(geometry_msgs::Twist start_vel, geometry_msgs::Twist end_vel);
			float calcEuclideanDiff(geometry_msgs::Pose point1, geometry_msgs::Pose point2);
			#pragma endregion
	};
}