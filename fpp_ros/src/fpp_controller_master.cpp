#include <fpp_ros/fpp_controller_master.h>

namespace fpp
{
    FPPControllerMaster::FPPControllerMaster(std::list<fpp_data_classes::RobotInfo> &robot_info_list,
                                             fpp_data_classes::RobotInfo *&robot_info,
                                             ros::NodeHandle &nh)
        : FPPControllerBase(robot_info_list, robot_info, nh)
    {
        this->initServices();
        this->initTopics();

        // Initialize formation planner with default values. Set lead vector when all robots are added and centroid can be calculated
        this->formation_contour_ = geometry_info::FormationContour(Eigen::Matrix<float, 2, 1>::Zero(), 0.0);
        
        for(fpp_data_classes::RobotInfo robot_info: this->robot_info_list_)
        {
            Eigen::Vector2f robot_pose;
            float yaw;
            this->getAMCLPose(robot_info.robot_namespace, robot_pose, yaw);
            
            geometry_info::RobotContour robot_contour;
            robot_contour = geometry_info::RobotContour(robot_pose, yaw, robot_info.robot_name);
            
            for(Eigen::Vector2f corner: robot_info.robot_outline)
            {
                robot_contour.addContourCornerGeometryCS(corner);
            }
            robot_contour.createContourEdges();

            this->robot_outline_list_.insert(std::pair<std::string, geometry_info::RobotContour>(robot_info.robot_name, robot_contour));
            this->formation_contour_.addRobotToFormation(robot_contour);
        }
        this->formation_contour_.updateFormationContour();

        // Call services
        fpp_helper::MinimalEnclosingCircle formation_outline_circle = fpp_helper::MinimalEnclosingCircle();
        std::vector<Eigen::Vector2f> robot_positions;
        formation_outline_circle.calcMinimalEnclosingCircle(this->formation_contour_.getCornerPointsWorldCS());
        fpp_msgs::DynReconfigure dyn_reconfig_msg;
        dyn_reconfig_msg.request.new_inflation_radius = formation_outline_circle.getCircleRadius();
        dyn_reconfig_msg.request.robot_namespace = this->robot_info_->robot_namespace;
        ros::Duration(0.1).sleep();
        this->dyn_rec_inflation_srv_client_.call(dyn_reconfig_msg);

        this->publishFootprint();
        
        this->initTimers();
    }

    void FPPControllerMaster::initServices()
    {
        this->dyn_rec_inflation_srv_client_ = this->nh_.serviceClient<fpp_msgs::DynReconfigure>("/dyn_reconfig_inflation");
        this->dyn_rec_inflation_srv_client_.waitForExistence();
    }

    void FPPControllerMaster::initTopics()
    {
        this->formation_footprint_pub_ = this->nh_.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10);
        while(this->formation_footprint_pub_.getNumSubscribers() < 1)
            ros::Duration(0.01).sleep();
    }

    void FPPControllerMaster::initTimers()
    {
        this->footprint_timer_ = this->nh_.createTimer(ros::Duration(1.0), &FPPControllerMaster::footprintTimerCallback, this);
    }


    void FPPControllerMaster::execute()
    {
        ROS_ERROR("execute");
    }


    geometry_msgs::PoseWithCovarianceStampedConstPtr FPPControllerMaster::getAMCLPose(std::string robot_namespace)
    {
        std::string amcl_pose_topic = robot_namespace + "/amcl_pose";
        geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose_ptr;
        robot_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(amcl_pose_topic);
        return robot_pose_ptr;
    }

    void FPPControllerMaster::getAMCLPose(std::string robot_namespace, Eigen::Vector2f &robot_pose, float &yaw)
    {
        geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose_ptr;
        robot_pose_ptr = this->getAMCLPose(robot_namespace);

        if(robot_pose_ptr == nullptr)
        {
            ROS_ERROR_STREAM("FormationPathPlanner: No message was received from the amcl_pose topic in the robot_namespace: " << robot_namespace);
            robot_pose << 0, 0;
            yaw = 0;
        }
        else
        {
            robot_pose << robot_pose_ptr->pose.pose.position.x, robot_pose_ptr->pose.pose.position.y;
            yaw = tf::getYaw(robot_pose_ptr->pose.pose.orientation);
        }
    }

    void FPPControllerMaster::updateFootprint()
    {
        for(fpp_data_classes::RobotInfo robot_info: this->robot_info_list_)
        {
            Eigen::Vector2f new_robot_pose;
            float new_rotation;
            this->getAMCLPose(robot_info.robot_namespace, new_robot_pose, new_rotation);

            this->formation_contour_.updateRobotPose(robot_info.robot_name, new_robot_pose, new_rotation);
        }
        this->formation_contour_.updateFormationContour();
    }

    void FPPControllerMaster::publishFootprint()
    {
        geometry_msgs::PolygonStamped formation_footprint_msg;
        formation_footprint_msg.header.frame_id = "map";
        formation_footprint_msg.header.stamp = ros::Time::now();

        std::vector<Eigen::Vector2f> formation_corner_points = this->formation_contour_.getCornerPointsWorldCS();
        for(Eigen::Vector2f corner: formation_corner_points)
        {
            geometry_msgs::Point32 corner_point;
            corner_point.x = corner[0];
            corner_point.y = corner[1];
            corner_point.z = 0.0;
            formation_footprint_msg.polygon.points.push_back(corner_point);
        }

        this->formation_footprint_pub_.publish(formation_footprint_msg);
    }

    void FPPControllerMaster::footprintTimerCallback(const ros::TimerEvent& e)
    {
        this->updateFootprint();
        this->publishFootprint();        
    }
}