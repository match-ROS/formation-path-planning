#include <relaxed_a_star/relaxed_a_star.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(relaxed_a_star::RelaxedAStar, mbf_costmap_core::CostmapPlanner)
PLUGINLIB_EXPORT_CLASS(relaxed_a_star::RelaxedAStar, nav_core::BaseGlobalPlanner)

namespace relaxed_a_star
{
     RelaxedAStar::RelaxedAStar()
     {
         ROS_ERROR("RELAXED A STAR CONSTRUCTOR");
     }

    uint32_t RelaxedAStar::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message)
    {
        ROS_ERROR("RELAXED A STAR MAKEPLAN");
    }

    bool RelaxedAStar::makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        ROS_ERROR("RELAXED A STAR MAKEPLAN NAV_CORE");
    }   

    bool RelaxedAStar::cancel()
    {
        ROS_ERROR("RELAXED A STAR CANCEL");
    }

    void RelaxedAStar::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ROS_ERROR("RELAXED A STAR INITIALIZE");
    }
}