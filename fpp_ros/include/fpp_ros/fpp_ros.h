#pragma once

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/SetBool.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <nav_core/base_global_planner.h>

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>

namespace fpp
{
    class FormationPathPlanner : public nav_core::BaseGlobalPlanner, public mbf_costmap_core::CostmapPlanner
    {
        public: 
            /**
             * @brief Default constructor of the Relaxed A Star class
             * 
             */
            FormationPathPlanner();

            /**
             * @brief Construct a new Relaxed A Star object
             * 
             * @param name The name of this planner
             * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
             */
            FormationPathPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

            // mbf_costmap_core::CostmapPlanner interface implementation
            /**
            * @brief Initialization function for the CostmapPlanner
            * @param name The name of this planner
            * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
            */
            void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

            /**
             * @brief Initialization method for the CostmapPlanner
             * 
             * @param name The name of this planner
             * @param costmap A pointer to the costmap that will be used for planning
             * @param global_frame Global frame of the costmap
             */
            void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

            /**
             * @brief Given a goal pose in the world, compute a plan
             * @param start The start pose 
             * @param goal The goal pose 
             * @param plan The plan... filled by the planner
             * @return True if a valid plan was found, false otherwise
             */
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

            /**
             * @brief Given a goal pose in the world, compute a plan
             * @param start The start pose
             * @param goal The goal pose
             * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
             *        in x and y before failing
             * @param plan The plan... filled by the planner
             * @param cost The cost for the the plan
             * @param message Optional more detailed outcome as a string
             * @return Result code as described on GetPath action result:
             *         SUCCESS         = 0
             *         1..9 are reserved as plugin specific non-error results
             *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
             *         CANCELED        = 51
             *         INVALID_START   = 52
             *         INVALID_GOAL    = 53
             *         NO_PATH_FOUND   = 54
             *         PAT_EXCEEDED    = 55
             *         EMPTY_PATH      = 56
             *         TF_ERROR        = 57
             *         NOT_INITIALIZED = 58
             *         INVALID_PLUGIN  = 59
             *         INTERNAL_ERROR  = 60
             *         71..99 are reserved as plugin specific errors
             */
            uint32_t makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                        double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                        std::string &message);

            /**
            * @brief Requests the planner to cancel, e.g. if it takes too much time.
            * @remark New on MBF API
            * @return True if a cancel has been successfully requested, false if not implemented.
            */
            bool cancel();

        protected:
            // ROS wrapper for the 2DCostmap object. Probably unnessecary to also store this next to the 2DCostmap object
            costmap_2d::Costmap2DROS *costmap_ros_;
            // This object is a pointer to the 2D costmap
            costmap_2d::Costmap2D *costmap_;

        private:
            bool initialized_;

            void getParams();

            std::string robot_name_;
            /**
             * @brief Global frame of the robot
             * 
             */
            std::string global_frame_;
            /**
             * @brief tf_prefix that was defined in the launch files for the robot
             * 
             */
            std::string tf_prefix_;

            // Parameter list
            // The default tolerance that is used if the tolerance of the received goal is not valid
            // Default: 0.2
            float default_tolerance_;

            // This list is initialized through the path planner config file
            // Contains all positions of every robot that is part of the formation
            std::map<std::string, geometry_msgs::Pose> robot_positions_;

            // Process information
            /**
             * @brief This value defines the size of the arrays that are the 1D-representation of the costmap
             * This is calculated by multiplying the width and height of the costmap
             * 
             */
            int array_size_;
            /**
             * @brief Storing the start position as pose from where the robot is starting
             * 
             */
            geometry_msgs::PoseStamped start_;
            /**
             * @brief Storing the goal position as pose where the robot should arrive after following the calculated path
             * 
             */
            geometry_msgs::PoseStamped goal_;
            
    };
}