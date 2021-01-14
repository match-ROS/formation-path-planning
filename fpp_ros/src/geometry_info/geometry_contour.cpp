#include <geometry_info/geometry_contour.h>

namespace geometry_info
{
    GeometryContour::GeometryContour(Eigen::Vector2d lead_vector_world_cs, float world_to_geometry_cs_rotation)
    {
        this->lead_vector_world_cs_= lead_vector_world_cs;
        this->world_to_geometry_cs_rotation_ = world_to_geometry_cs_rotation;

        this->tf_geometry_to_world_cs_ = this->createTransformationMatrix(this->lead_vector_world_cs_,
                                                                          this->world_to_geometry_cs_rotation_);
        this->tf_world_to_geometry_cs_ = this->tf_geometry_to_world_cs_.inverse();
    }

    void GeometryContour::addContourCornerWorldCS(Eigen::Vector2d corner_world_cs)
    {
        Eigen::Vector2d corner_geometry_cs = this->transformWorldToGeometryCS(corner_world_cs);
        this->corner_points_geometry_cs_.push_back(corner_geometry_cs);
    }

    void GeometryContour::createContourEdges()
    {
        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            Eigen::Vector2d start_point_geometry_cs = this->corner_points_geometry_cs_[corner_counter];
            Eigen::Vector2d end_point_geometry_cs;
            if(corner_counter + 1 == this->corner_points_geometry_cs_.size())
            {
                end_point_geometry_cs = this->corner_points_geometry_cs_[0];
            }
            else
            {
                end_point_geometry_cs = this->corner_points_geometry_cs_[corner_counter + 1];
            }
            geometry_info::EdgeInfo edge_info = geometry_info::EdgeInfo(start_point_geometry_cs, end_point_geometry_cs);
            this->edge_list_geometry_cs_.push_back(edge_info);
        }
    }

    Eigen::Vector2d GeometryContour::transformWorldToGeometryCS(Eigen::Vector2d world_cs)
    {
        Eigen::Vector3d extended_world_cs;
        extended_world_cs << world_cs, 1;
        return this->tf_world_to_geometry_cs_ * extended_world_cs;
    }

    Eigen::Vector2d GeometryContour::transformGeometryToWorldCS(Eigen::Vector2d geometry_cs)
    {
        Eigen::Vector3d extended_geometry_cs;
        extended_geometry_cs << geometry_cs, 1;
        Eigen::Vector3d extended_world_cs;
        extended_world_cs = this->tf_geometry_to_world_cs_ * extended_geometry_cs;
        return extended_world_cs.head<2>();
    }


    Eigen::Matrix<float, 3, 3> GeometryContour::createTransformationMatrix(Eigen::Vector2d lead_vector_world_cs, float rotation)
    {
        Eigen::Matrix<float, 3, 3> transformation_matrix;
        transformation_matrix << cos(rotation), -sin(rotation), lead_vector_world_cs_[0],
            sin(rotation), cos(rotation), lead_vector_world_cs_[1],
            0, 0, 1;
        return transformation_matrix;
    }


}