#include <geometry_info/formation_contour.h>

namespace geometry_info
{
    FormationContour::FormationContour(Eigen::Vector2d lead_vector_world_cs, float world_to_geometry_cs_rotation)
        : GeometryContour(lead_vector_world_cs_, world_to_geometry_cs_rotation)
    {

    }

    void FormationContour::exeGiftWrappingAlg()
    {
        if(this->corner_points_geometry_cs_.size() > 2)
        {
            return; // Error. With only 2 corners there can not be an area between the points
        }

        // Start with finding the most left poit of the point cloud
        int index_leftmost_point = 0;
        Eigen::Vector2d leftmost_point_geometry_cs = this->corner_points_geometry_cs_[0]; // Initialize with first point from list
        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            if(this->corner_points_geometry_cs_[corner_counter][0] < leftmost_point_geometry_cs[0])
            {
                leftmost_point_geometry_cs = this->corner_points_geometry_cs_[corner_counter];
                index_leftmost_point = corner_counter;
            }
        }

        std::vector<Eigen::Vector2d> corners_to_wrap_geometry_cs = this->corner_points_geometry_cs_;
        std::vector<Eigen::Vector2d> wrapped_contour_corners_geometry_cs;
        wrapped_contour_corners_geometry_cs.push_back(leftmost_point_geometry_cs);
        corners_to_wrap_geometry_cs.erase(corners_to_wrap_geometry_cs.begin() + index_leftmost_point);

        // Get next point as long as the start point was not reached again
        while(wrapped_contour_corners_geometry_cs.size() <= 1 && wrapped_contour_corners_geometry_cs.back() != leftmost_point_geometry_cs)
        {
            int next_wrapping_point_index;
            Eigen::Vector2d next_wrapping_point_geometry_cs;
            next_wrapping_point_index = this->calcNextWrappingPoint(wrapped_contour_corners_geometry_cs.back(),
                                                                    corners_to_wrap_geometry_cs,
                                                                    next_wrapping_point_geometry_cs);
            wrapped_contour_corners_geometry_cs.push_back(next_wrapping_point_geometry_cs);
            corners_to_wrap_geometry_cs.erase(corners_to_wrap_geometry_cs.begin() + next_wrapping_point_index);                                                                    
        }

        this->corner_points_geometry_cs_ = wrapped_contour_corners_geometry_cs;
        this->createContourEdges();
    }

    int FormationContour::calcNextWrappingPoint(Eigen::Vector2d current_wrapping_point_geometry_cs,
                                                std::vector<Eigen::Vector2d> corners_to_wrap_geometry_cs,
                                                Eigen::Vector2d &next_wrapping_point_geometry_cs)
    {
        int next_wrapping_point_index = 0;
        next_wrapping_point_geometry_cs = corners_to_wrap_geometry_cs[next_wrapping_point_index];
        float max_radian_next_wrapping_point = this->calcTan2(current_wrapping_point_geometry_cs,
                                                              next_wrapping_point_geometry_cs);
        for(int corner_to_wrap_counter = 0; corner_to_wrap_counter < corners_to_wrap_geometry_cs.size(); corner_to_wrap_counter++)
        {
            float radian_next_wrapping_point = this->calcTan2(current_wrapping_point_geometry_cs,
                                                              corners_to_wrap_geometry_cs[corner_to_wrap_counter]);
            if(max_radian_next_wrapping_point < radian_next_wrapping_point)
            {
                max_radian_next_wrapping_point = radian_next_wrapping_point;
                next_wrapping_point_index = corner_to_wrap_counter;
                next_wrapping_point_geometry_cs = corners_to_wrap_geometry_cs[corner_to_wrap_counter];
            }                                                              
        }
        return next_wrapping_point_index;
    }

    float FormationContour::calcTan2(Eigen::Vector2d start_point_geometry_cs, Eigen::Vector2d end_point_geometry_cs)
    {
        return std::atan2(end_point_geometry_cs[1] - start_point_geometry_cs[1],
                          end_point_geometry_cs[0] - start_point_geometry_cs[0]);
    }
}