#include <fpp_ros/geometry_info/minimal_enclosing_circle.h>

namespace geometry_info
{
    void MinimalEnclosingCircle::calcMinimalEnclosingCircle(const std::vector<Eigen::Vector2f> &points_to_enclose)
    {
        std::vector<Eigen::Vector2f> points_to_enclose_copy = points_to_enclose;
        CircleInfo circle_info = this->exeWetzlAlg(points_to_enclose_copy, { }, points_to_enclose.size());

        this->enclosed_points_ = points_to_enclose;
        this->circle_radius_ = circle_info.radius;
        this->circle_centre_ = circle_info.centre;

        // DEBUGGING
        std::cout << "Minimal Circle | x: " << this->circle_centre_[0] << " , y: " << this->circle_centre_[1] << " , radius: " << this->circle_radius_ << "\n";
        for(Eigen::Vector2f point: this->circle_defining_points_)
        {
            std::cout << "Circle defining points: | x: " << point[0] << " , y: " << point[1] << "\n";
        }
    }

    void MinimalEnclosingCircle::calcMinimalEnclosingCircle(const Eigen::Vector2f circle_centre, const std::vector<Eigen::Vector2f> &points_to_enclose)
    {
        this->circle_centre_ = circle_centre;
        
        float max_radius = points_to_enclose[0].norm();

        // Get point with max distance to circle centre. This point defines the radius
        for(const Eigen::Vector2f &point: points_to_enclose)
        {
            float new_radius = point.norm();
            if(max_radius < new_radius)
            {
                max_radius = new_radius;
            }
        }

        this->circle_radius_ = max_radius;

        // DEBUGGING
        std::cout << "Minimal Circle | x: " << this->circle_centre_[0] << " , y: " << this->circle_centre_[1] << " , radius: " << this->circle_radius_ << "\n";
    }

    CircleInfo MinimalEnclosingCircle::exeWetzlAlg(std::vector<Eigen::Vector2f> &points_to_enclose,
                                                   std::vector<Eigen::Vector2f> circle_defining_points,
                                                   int points_to_go)
    {
        if(points_to_go == 0 || circle_defining_points.size() == 3)
        {
            return this->createCircle(circle_defining_points);
        }

        // Select random point
        int random_point_index = rand() % points_to_go;
        Eigen::Vector2f point = points_to_enclose[random_point_index];

        // Remove selected point from list
        points_to_enclose.erase(points_to_enclose.begin() + random_point_index);

        CircleInfo circle_info = this->exeWetzlAlg(points_to_enclose, circle_defining_points, points_to_go - 1);
        
        if(this->isPointEnclosed(circle_info, point))
        {
            return circle_info;
        }
        circle_defining_points.push_back(point);
        this->circle_defining_points_ = circle_defining_points; // This will get overwritten every time but the last one will stay

        return this->exeWetzlAlg(points_to_enclose, circle_defining_points, points_to_go - 1);
    }

    CircleInfo MinimalEnclosingCircle::createCircle(std::vector<Eigen::Vector2f> circle_defining_points)
    {
        CircleInfo circle_info;
        if(circle_defining_points.size() >= 4)
        {
            std::cout << "MinimalEnclosingCircle: ERROR! To many points in MinimalEnclosingCircle::createCircle() method. \n";
        }
        else if(circle_defining_points.size() == 0)
        {
            circle_info.centre = Eigen::Matrix<float, 2, 1>::Zero();
            circle_info.radius = 0.0;
        }
        else if(circle_defining_points.size() == 1)
        {
            circle_info.centre = circle_defining_points[0];
            circle_info.radius = 0.0;
        }
        else if(circle_defining_points.size() == 2)
        {
            circle_info = this->createCircle(circle_defining_points[0], circle_defining_points[1]);
        }
        else if(circle_defining_points.size() == 3)
        {
            // Check if smallest circle can be created with only two points
            for(int outer_counter = 0; outer_counter < 3; outer_counter++)
            {
                for(int inner_counter = outer_counter + 1; inner_counter < 3; inner_counter++)
                {
                    CircleInfo two_point_circle_info = this->createCircle(circle_defining_points[outer_counter],
                                                                          circle_defining_points[inner_counter]);
                    if(this->arePointsEnclosed(two_point_circle_info, circle_defining_points))
                        return two_point_circle_info;
                }
            }

            // Smallest circle can only be created with three points
            circle_info = this->createCircle(circle_defining_points[0],
                                             circle_defining_points[1],
                                             circle_defining_points[2]);
        }

        return circle_info;
    }

    CircleInfo MinimalEnclosingCircle::createCircle(Eigen::Vector2f point1, Eigen::Vector2f point2)
    {
        CircleInfo circle_info;
        circle_info.centre = this->calcCentreOfVector(point1, point2);
        circle_info.radius = 0.5 * this->calcDistance(point1, point2);
        return circle_info;
    }

    CircleInfo MinimalEnclosingCircle::createCircle(Eigen::Vector2f point1, Eigen::Vector2f point2, Eigen::Vector2f point3)
    {
        Eigen::Vector2f vector_centre1 = this->calcCentreOfVector(this->circle_defining_points_[1], this->circle_defining_points_[0]);
        Eigen::Vector2f orthogonal_vector1 = this->calcOrthogonalVector(this->circle_defining_points_[0] - this->circle_defining_points_[1]);
        Eigen::Vector2f vector_centre2 = this->calcCentreOfVector(this->circle_defining_points_[1], this->circle_defining_points_[2]);
        Eigen::Vector2f orthogonal_vector2 = this->calcOrthogonalVector(this->circle_defining_points_[2] - this->circle_defining_points_[1]);

        CircleInfo circle_info;
        circle_info.centre = this->calcVectorLineIntersectionPoint(vector_centre1, orthogonal_vector1,
                                                                        vector_centre2, orthogonal_vector2);
        circle_info.radius = this->calcDistance(this->circle_defining_points_[0], this->circle_centre_);
		
		return circle_info;
    }

    double MinimalEnclosingCircle::calcDistance(Eigen::Vector2f first_point, Eigen::Vector2f second_point)
    {
        Eigen::Vector2f diff_vector = second_point - first_point;
        return diff_vector.norm();
    }

    Eigen::Vector2f MinimalEnclosingCircle::calcCentreOfVector(Eigen::Vector2f first_point, Eigen::Vector2f second_point)
    {
        Eigen::Vector2f diff_vector = second_point - first_point;
        Eigen::Vector2f relative_circle_centre = 0.5 * diff_vector;
        return first_point + relative_circle_centre;
    }

    Eigen::Vector2f MinimalEnclosingCircle::calcOrthogonalVector(Eigen::Vector2f vector)
    {
        // Calculating the orthogonal vector is just swaping x and y and after that inverting the x coordinate
        Eigen::Vector2f orthognal_vector;
        orthognal_vector[0] = -vector[1];
        orthognal_vector[1] = vector[0];
        return orthognal_vector;
    }

    Eigen::Vector2f MinimalEnclosingCircle::calcVectorLineIntersectionPoint(Eigen::Vector2f lead_vector1,
                                                                            Eigen::Vector2f direction_vector1,
                                                                            Eigen::Vector2f lead_vector2,
                                                                            Eigen::Vector2f direction_vector2)
    {
        double numerator = ((lead_vector2[1] * direction_vector2[0]) + (lead_vector1[0] * direction_vector2[1]) -
                            (lead_vector2[0] * direction_vector2[1]) - (lead_vector1[1] * direction_vector2[0]));

        double denominator = (direction_vector1[1] * direction_vector2[0]) - (direction_vector1[0] * direction_vector2[1]);

        double factor = numerator / denominator;

        return lead_vector1 + (factor * direction_vector1);
    }

    bool MinimalEnclosingCircle::isPointEnclosed(const CircleInfo &circle_info, const Eigen::Vector2f &point)
    {
        return (this->calcDistance(circle_info.centre, point) <= circle_info.radius);
    }

    bool MinimalEnclosingCircle::arePointsEnclosed(const CircleInfo &circle_info, const std::vector<Eigen::Vector2f> &points)
    {
        for(const Eigen::Vector2f &point: points)
        {
            if(!this->isPointEnclosed(circle_info, point))
                return false;
        }
        return true;
    }

    double MinimalEnclosingCircle::getCircleRadius()
    {
        return this->circle_radius_;
    }

    Eigen::Vector2f MinimalEnclosingCircle::getCircleCentre()
    {
        return this->circle_centre_;
    }

    std::vector<Eigen::Vector2f> MinimalEnclosingCircle::getCircleDefiningPoints()
    {
        return this->circle_defining_points_;
    }

    std::vector<Eigen::Vector2f> MinimalEnclosingCircle::getEnclosedPoints()
    {
        return this->enclosed_points_;
    }
}