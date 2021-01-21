#include <fpp_ros/geometry_info/formation_contour.h>

namespace geometry_info
{
    FormationContour::FormationContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation)
        : GeometryContour(lead_vector_world_cs_, world_to_geometry_cs_rotation)
    {

    }

    void FormationContour::exeGiftWrappingAlg()
    {
        if(this->corner_points_geometry_cs_.size() < 2)
        {
            return; // Error. With only 2 corners there can not be an area between the points
        }

        // Start with finding the most left poit of the point cloud
        int index_lowest_point = 0;
        Eigen::Vector2f lowest_point_geometry_cs = this->corner_points_geometry_cs_[0]; // Initialize with first point from list
        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            if(this->corner_points_geometry_cs_[corner_counter][1] < lowest_point_geometry_cs[1])
            {
                lowest_point_geometry_cs = this->corner_points_geometry_cs_[corner_counter];
                index_lowest_point = corner_counter;
            }
        }

        std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs = this->corner_points_geometry_cs_;
        std::vector<Eigen::Vector2f> wrapped_contour_corners_geometry_cs;
        wrapped_contour_corners_geometry_cs.push_back(lowest_point_geometry_cs);
        
        float previous_radian = 2*M_PI; // Init with pi because no point can be below this one. After this point we rotate the line clockwise to find the next point
        // Get next point as long as the start point was not reached again
        while(wrapped_contour_corners_geometry_cs.size() <= 1 || wrapped_contour_corners_geometry_cs.back() != lowest_point_geometry_cs)
        {
            int next_wrapping_point_index;
            Eigen::Vector2f next_wrapping_point_geometry_cs;
            
            next_wrapping_point_index = this->calcNextWrappingPoint(wrapped_contour_corners_geometry_cs.back(),
                                                                    corners_to_wrap_geometry_cs,
                                                                    next_wrapping_point_geometry_cs,
                                                                    previous_radian);
            wrapped_contour_corners_geometry_cs.push_back(next_wrapping_point_geometry_cs);
            corners_to_wrap_geometry_cs.erase(corners_to_wrap_geometry_cs.begin() + next_wrapping_point_index);       
        }
        this->corner_points_geometry_cs_ = wrapped_contour_corners_geometry_cs;
        this->createContourEdges();
    }

    void FormationContour::addRobotToFormation(geometry_info::RobotContour robot_to_add)
    {
        if (std::find(this->robot_contours_.begin(), this->robot_contours_.end(), robot_to_add) == this->robot_contours_.end())
        {
            this->robot_contours_.push_back(robot_to_add);
        }
        else
        {
            std::cout << "FormationContour: Robot already exists in formation.";
        }
    }

    void FormationContour::updateRobotPose(std::string robot_name,
                                           Eigen::Vector2f new_robot_pose_world_cs,
                                           float new_rotation_world_to_geometry_cs)
    {
        for(RobotContour &robot_contour : this->robot_contours_)
        {
            if(robot_contour.getRobotName() == robot_name)
            {
                robot_contour.move_contour(new_robot_pose_world_cs, new_rotation_world_to_geometry_cs);
            }
        }
    }

    void FormationContour::updateFormationContour()
    {
        this->corner_points_geometry_cs_.clear(); // Delete all current corner points

        // Add all corners of current robots and then exe gift wrapping algorithm
        for(RobotContour &robot_contour: this->robot_contours_)
        {
            this->addContourCornersWorldCS(robot_contour.getCornerPointsWorldCS());
        }
        this->exeGiftWrappingAlg();
    }

    int FormationContour::calcNextWrappingPoint(Eigen::Vector2f current_wrapping_point_geometry_cs,
                                                std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs,
                                                Eigen::Vector2f &next_wrapping_point_geometry_cs,
                                                float &max_radian)
    {
        int next_wrapping_point_index = 0;
        float next_max_radian;
        float smallest_diff_to_max_radian = 2*M_PI; // This is the max value and should never occure because I look at the diff

        for(int corner_to_wrap_counter = 0; corner_to_wrap_counter < corners_to_wrap_geometry_cs.size(); corner_to_wrap_counter++)
        {
            if(current_wrapping_point_geometry_cs == corners_to_wrap_geometry_cs[corner_to_wrap_counter])
            {
                continue;
            }
            float radian_next_wrapping_point = this->calcTan2(current_wrapping_point_geometry_cs,
                                                              corners_to_wrap_geometry_cs[corner_to_wrap_counter]);
            radian_next_wrapping_point = radian_next_wrapping_point + M_PI; // So I dont get a value area from -pi to pi.
            float next_diff = max_radian - radian_next_wrapping_point;

            if(radian_next_wrapping_point <= max_radian && smallest_diff_to_max_radian > next_diff)
            {
                smallest_diff_to_max_radian = next_diff;
                next_wrapping_point_index = corner_to_wrap_counter;
                next_max_radian = radian_next_wrapping_point;
                next_wrapping_point_geometry_cs = corners_to_wrap_geometry_cs[corner_to_wrap_counter];
            }                                                              
        }
        max_radian = next_max_radian;
        return next_wrapping_point_index;
    }

    float FormationContour::calcTan2(Eigen::Vector2f start_point_geometry_cs, Eigen::Vector2f end_point_geometry_cs)
    {
        return std::atan2(end_point_geometry_cs[1] - start_point_geometry_cs[1],
                          end_point_geometry_cs[0] - start_point_geometry_cs[0]);
    }
}