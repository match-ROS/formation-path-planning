#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/SetBool.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <cmath>

namespace relaxed_a_star
{
    /**
     * @brief Struct that acts as a container for storing the call with the according f_cost value
     * Contains overwriten > and < operators for multiset comparator functions
     * 
     */
    struct Cell
    {
        int cell_index;
        float f_cost;

        bool operator<(const Cell& rhs) const
        { 
            return this->f_cost < rhs.f_cost;
        }

        bool operator>(const Cell& rhs) const
        {
            return this->f_cost > rhs.f_cost;
        }
    };

    struct tempStruct
    {
        int array_index;
        int x;
        int y;
    };

    enum NeighborType
    {
        FourWay = 4,
        EightWay = 8
    };

    class RelaxedAStar : public nav_core::BaseGlobalPlanner, public mbf_costmap_core::CostmapPlanner
    {
        public:
            RelaxedAStar();
            RelaxedAStar(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

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

            bool initialized_;

            costmap_2d::Costmap2DROS *costmap_ros_;
            costmap_2d::Costmap2D *costmap_;
            std::shared_ptr<bool[]> occupancy_map_; // One dimensional represantation of the map. True = occupied, false = free

            ros::Publisher plan_publisher_;
            ros::Publisher planning_points_orientation_publisher_;
            ros::Publisher debug_publisher_;
            ros::Publisher marker_array_publisher_;
            ros::ServiceServer trigger_costmap_check_service_;
            ros::Subscriber costmap_sub_;

        private:
            bool service_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
            void costmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg);

            void findPlan(int array_start_cell, int array_goal_cell, std::shared_ptr<float[]> g_score);
            std::vector<int> createPlan(int array_start_cell, int array_goal_cell, std::shared_ptr<float[]> g_score);

            /**
             * @brief Method for calulcating the g score for the target cell.
             * 
             * @param current_g_cost g score for the current cell
             * @param array_current_cell Index for the current cell in the one dimensional representation of the costmap
             * @param array_target_cell Index for the target cell in the one dimensional representation of the costmap
             * @return float 
             */
            float calcGCost(int current_g_cost, int array_current_cell, int array_target_cell);

            /**
             * @brief Calculates the heuristic cost from current position to the goal_cell.
             * Currently the cost will be approximated with the euclidean distance.
             * 
             * @param map_current_cell Current cell from where the heuristic cost should be calculated to the goal
             * @param map_goal_cell Goal of the path planning
             * @return float 
             */
            float calcHCost(int* map_current_cell, int* map_goal_cell);
            float calcHCost(int array_current_cell, int array_goal_cell);

            float calcFCost(float current_g_score, int array_current_cell, int array_goal_cell);


            /**
             * @brief Gets the index of the 2D position in the 1D representing array
             * Here the costmap is represented by a 1 dimensional array.
             * 
             * @param map_point Array that contains x and y index of the costmap
             * @return int Cell index in one dimensional representation
             */
            int getArrayIndexByCostmapCell(int *map_cell);

            /**
             * @brief Gets the index of the 2D position in the 1D representing array
             * Here the costmap is represented by a 1 dimensional array.
             * 
             * @param map_cell_x X coordinate of the point in the map as int
             * @param map_cell_y Y coordinate of the point in the map as int
             * @return int Cell index in one dimensional representation
             */
            int getArrayIndexByCostmapCell(int map_cell_x, int map_cell_y);

            /**
             * @brief Gets the index of the 1D array in the 2D costmap
             * Here the costmap is represented by a 1 dimensional array.
             * 
             * @param array_index Index of the position in the one dimensional array
             * @param map_cell Cell in the two dimensional costmap
             */
            void getCostmapPointByArrayIndex(int array_index, int *map_cell);

            /**
             * @brief Gets all free neighbor cells adjacent to the current cell
             * 
             * @param array_current_cell 
             * @return std::vector<int> 
             */
            std::vector<int> getFreeNeighborCells(int array_current_cell);

            int getMinGScoreNeighborCell(int array_current_cell, std::shared_ptr<float[]> g_score);

            void initializeOccupancyMap();
            bool isCellFree(int array_cell_index);

            float calcMoveCost(int array_current_cell, int array_target_cell);
            float calcMoveCost(int* map_current_cell, int* map_target_cell);
            float calcMoveCost(int map_current_cell_x, int map_current_cell_y, int map_target_cell_x, int map_target_cell_y);

            void publishPlan(std::vector<geometry_msgs::PoseStamped> &plan);
            void publishOccupancyGrid();

            std::string global_frame_;
            std::string tf_prefix_;

            // Parameter list
            float default_tolerance_;
            NeighborType neighbor_type_;
            int free_cell_threshhold_;

            // Process information
            int array_size_;
    };
}
