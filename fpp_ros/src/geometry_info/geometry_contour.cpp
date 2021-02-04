#include <fpp_ros/geometry_info/geometry_contour.h>

namespace geometry_info
{
    GeometryContour::GeometryContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation)
    {
        this->lead_vector_world_cs_= lead_vector_world_cs;
        this->world_to_geometry_cs_rotation_ = world_to_geometry_cs_rotation;

        this->tf_geometry_to_world_cs_ = this->createTransformationMatrix(lead_vector_world_cs,
                                                                          world_to_geometry_cs_rotation);
        this->tf_world_to_geometry_cs_ = this->tf_geometry_to_world_cs_.inverse();
    }

    void GeometryContour::addContourCornerWorldCS(Eigen::Vector2f corner_world_cs)
    {
        Eigen::Vector2f corner_geometry_cs = this->transformWorldToGeometryCS(corner_world_cs);
        this->corner_points_geometry_cs_.push_back(corner_geometry_cs);
    }

    void GeometryContour::addContourCornerGeometryCS(Eigen::Vector2f corner_geometry_cs)
    {
        this->corner_points_geometry_cs_.push_back(corner_geometry_cs);
    }

    void GeometryContour::addContourCornersWorldCS(std::vector<Eigen::Vector2f> corner_list_world_cs)
    {
        for(Eigen::Vector2f corner_world_cs: corner_list_world_cs)
        {
            Eigen::Vector2f corner_geometry_cs = this->transformWorldToGeometryCS(corner_world_cs);
            if (std::find(this->corner_points_geometry_cs_.begin(),
                          this->corner_points_geometry_cs_.end(),
                          corner_geometry_cs) == this->corner_points_geometry_cs_.end())
            {
                this->corner_points_geometry_cs_.push_back(corner_geometry_cs);
            }
            else
            {
                std::cout << "GeometryContour::addContourCornersWorldCS: Corner point was already added, please check that all corners are different." << "\n";
            }
        }
    }

    void GeometryContour::createContourEdges()
    {
        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            Eigen::Vector2f start_point_geometry_cs = this->corner_points_geometry_cs_[corner_counter];
            Eigen::Vector2f end_point_geometry_cs;
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

    float GeometryContour::calcArea()
    {
        float area = 0.0;
        
        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            float part_area = 0.0;
            if((corner_counter + 1) == this->corner_points_geometry_cs_.size())
            {
                part_area = ((this->corner_points_geometry_cs_[corner_counter][0] *
                              this->corner_points_geometry_cs_[0][1]) -
                             (this->corner_points_geometry_cs_[0][0] *
                              this->corner_points_geometry_cs_[corner_counter][1]));
            }
            else
            {
                part_area = ((this->corner_points_geometry_cs_[corner_counter][0] *
                              this->corner_points_geometry_cs_[corner_counter + 1][1]) -
                             (this->corner_points_geometry_cs_[corner_counter + 1][0] *
                              this->corner_points_geometry_cs_[corner_counter][1]));
            }
            area = area + part_area;
        }

        area = 0.5 * area;

        return area;
    }

    float GeometryContour::calcAbsArea()
    {
        return std::abs(this->calcArea());
    }

    Eigen::Vector2f GeometryContour::calcCentroidGeometryCS()
    {
        float geometry_area = this->calcArea();

        float x_centroid = 0.0;
        float y_centroid = 0.0;

        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            float x_first_factor = 0.0;
            float y_first_factor = 0.0;
            float second_factor = 0.0;
            if((corner_counter + 1) == this->corner_points_geometry_cs_.size())
            {
                second_factor = ((this->corner_points_geometry_cs_[corner_counter][0] *
                                  this->corner_points_geometry_cs_[0][1]) -
                                 (this->corner_points_geometry_cs_[0][0] *
                                  this->corner_points_geometry_cs_[corner_counter][1]));
                x_first_factor = (this->corner_points_geometry_cs_[corner_counter][0] +
                                  this->corner_points_geometry_cs_[0][0]);
                y_first_factor = (this->corner_points_geometry_cs_[corner_counter][1] +
                                  this->corner_points_geometry_cs_[0][1]);
            }
            else
            {
                second_factor = ((this->corner_points_geometry_cs_[corner_counter][0] *
                                  this->corner_points_geometry_cs_[corner_counter + 1][1]) -
                                 (this->corner_points_geometry_cs_[corner_counter + 1][0] *
                                  this->corner_points_geometry_cs_[corner_counter][1]));
                x_first_factor = (this->corner_points_geometry_cs_[corner_counter][0] +
                                  this->corner_points_geometry_cs_[corner_counter + 1][0]);
                y_first_factor = (this->corner_points_geometry_cs_[corner_counter][1] +
                                  this->corner_points_geometry_cs_[corner_counter + 1][1]);
            }
            
            x_centroid = x_centroid + (x_first_factor * second_factor);
            y_centroid = y_centroid + (y_first_factor * second_factor);
        }

        x_centroid = (1 / (6 * geometry_area)) * x_centroid;
        y_centroid = (1 / (6 * geometry_area)) * y_centroid;

        Eigen::Vector2f centroid_geometry_cs;
        centroid_geometry_cs << x_centroid, y_centroid;

        return centroid_geometry_cs;
    }

    Eigen::Vector2f GeometryContour::calcCentroidWorldCS()
    {
        Eigen::Vector2f centroid_geometry_cs = this->calcCentroidGeometryCS();
        Eigen::Vector2f centroid_world_cs = this->transformGeometryToWorldCS(centroid_geometry_cs);
        return centroid_world_cs;
    }

    Eigen::Vector2f GeometryContour::transformWorldToGeometryCS(Eigen::Vector2f world_cs)
    {
        Eigen::Vector3f extended_world_cs;
        extended_world_cs << world_cs, 1;
        Eigen::Vector3f extended_geometry_cs;
        extended_geometry_cs = this->tf_world_to_geometry_cs_ * extended_world_cs;
        return extended_geometry_cs.head<2>();
    }

    Eigen::Vector2f GeometryContour::transformGeometryToWorldCS(Eigen::Vector2f geometry_cs)
    {
        Eigen::Vector3f extended_geometry_cs;
        extended_geometry_cs << geometry_cs, 1;
        Eigen::Vector3f extended_world_cs;
        extended_world_cs = this->tf_geometry_to_world_cs_ * extended_geometry_cs;
        return extended_world_cs.head<2>();
    }

    std::vector<Eigen::Vector2f> GeometryContour::getCornerPointsWorldCS()
    {
        std::vector<Eigen::Vector2f> corner_points_world_cs;
        for(Eigen::Vector2f corner_geometry_cs : this->corner_points_geometry_cs_)
        {
            Eigen::Vector2f corner_world_cs = this->transformGeometryToWorldCS(corner_geometry_cs);
            corner_points_world_cs.push_back(corner_world_cs);
        }
        return corner_points_world_cs;
    }

	Eigen::Matrix<float, 3, 3> GeometryContour::getTFWorldToGeometryCS()
	{
		return this->tf_world_to_geometry_cs_;
	}

	Eigen::Matrix<float, 3, 3> GeometryContour::getTFGeometryToWorldCS()
	{
		return this->tf_geometry_to_world_cs_;
	}

	Eigen::Vector2f GeometryContour::getLeadVectorWorldCS()
	{
		return this->lead_vector_world_cs_;
	}

	float GeometryContour::getWorldToGeometryCSRotation()
	{
		return this->world_to_geometry_cs_rotation_;
	}

    void GeometryContour::moveCoordinateSystem(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation)
    {
        Eigen::Matrix<float, 3, 3> new_tf_geometry_to_world_cs = this->createTransformationMatrix(new_lead_vector_world_cs,
                                                                                       new_cs_rotation);
        Eigen::Matrix<float, 3, 3> new_tf_world_to_geometry_cs = new_tf_geometry_to_world_cs.inverse();

        Eigen::Matrix<float, 3, 3> tf_old_to_new_geometry_cs;
        tf_old_to_new_geometry_cs = new_tf_world_to_geometry_cs * this->tf_geometry_to_world_cs_;

        std::vector<Eigen::Vector2f> corner_point_list_new_geometry_cs;
        for(Eigen::Vector2f corner_old_geometry_cs : this->corner_points_geometry_cs_)
        {
            Eigen::Vector3f corner_extended_old_geometry_cs;
            Eigen::Vector3f corner_extended_new_geometry_cs;
            
            corner_extended_old_geometry_cs << corner_old_geometry_cs, 1;
            corner_extended_new_geometry_cs = tf_old_to_new_geometry_cs * corner_extended_old_geometry_cs;
            corner_point_list_new_geometry_cs.push_back(corner_extended_new_geometry_cs.head<2>());
        }
        
        this->lead_vector_world_cs_ = new_lead_vector_world_cs;
        this->world_to_geometry_cs_rotation_ = new_cs_rotation;
        this->tf_geometry_to_world_cs_ = new_tf_geometry_to_world_cs;
        this->tf_world_to_geometry_cs_ = new_tf_world_to_geometry_cs;

        this->corner_points_geometry_cs_.clear();
        this->corner_points_geometry_cs_.insert(this->corner_points_geometry_cs_.end(),
                                                corner_point_list_new_geometry_cs.begin(),
                                                corner_point_list_new_geometry_cs.end());
        this->createContourEdges();
    }

    void GeometryContour::moveContour(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation)
    {
        Eigen::Matrix<float, 3, 3> new_tf_geometry_to_world_cs = this->createTransformationMatrix(new_lead_vector_world_cs, new_cs_rotation);
        Eigen::Matrix<float, 3, 3> new_tf_world_to_geometry_cs = new_tf_geometry_to_world_cs.inverse();

        this->lead_vector_world_cs_ = new_lead_vector_world_cs;
        this->world_to_geometry_cs_rotation_ = new_cs_rotation;
        this->tf_geometry_to_world_cs_ = new_tf_geometry_to_world_cs;
        this->tf_world_to_geometry_cs_ = new_tf_world_to_geometry_cs;
    }

    Eigen::Matrix<float, 3, 3> GeometryContour::createTransformationMatrix(Eigen::Vector2f lead_vector_world_cs, float rotation)
    {
        Eigen::Matrix<float, 3, 3> transformation_matrix;
        transformation_matrix << cos(rotation), -sin(rotation), lead_vector_world_cs[0],
            sin(rotation), cos(rotation), lead_vector_world_cs[1],
            0, 0, 1;
        return transformation_matrix;
    }

    bool operator==(const GeometryContour &lhs, const GeometryContour &rhs)
    {
        return std::equal(lhs.corner_points_geometry_cs_.begin(), lhs.corner_points_geometry_cs_.end(),
                          rhs.corner_points_geometry_cs_.begin(), rhs.corner_points_geometry_cs_.end()) &&
               lhs.tf_geometry_to_world_cs_ == rhs.tf_geometry_to_world_cs_;
    }
}