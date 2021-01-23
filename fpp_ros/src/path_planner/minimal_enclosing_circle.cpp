#include <fpp_ros/path_planner/minimal_enclosing_circle.h>

namespace fpp_helper
{
    MinimalEnclosingCircle::MinimalEnclosingCircle()
    {

    }

    void MinimalEnclosingCircle::calcMinimalEnclosingCircle(std::vector<Eigen::Vector2f> points_to_enclose)
    {
        this->enclosed_points_.push_back(points_to_enclose.back());
        this->circle_defining_points_.push_back(points_to_enclose.back());
        points_to_enclose.pop_back();
        this->enclosed_points_.push_back(points_to_enclose.back());
        this->circle_defining_points_.push_back(points_to_enclose.back());
        points_to_enclose.pop_back();

        // Initialize minimal circle with two points
        this->updateCircle();

        if(points_to_enclose.size() != 0)
        {
            for(Eigen::Vector2f point: points_to_enclose)
            {
                if(std::find(this->enclosed_points_.begin(), this->enclosed_points_.end(), point) == this->enclosed_points_.end())
                {
                    this->enclosed_points_.push_back(point);
                }

                double distance_to_centre = this->calcDistance(this->circle_centre_, point);
                if(distance_to_centre > this->circle_radius_)
                {
                    this->circle_defining_points_.push_back(point);
                    this->findNewSmallestCircle();
                }
            }
        }

        std::cout << "Minimal Circle | x: " << this->circle_centre_[0] << " , y: " << this->circle_centre_[1] << " , radius: " << this->circle_radius_ << "\n";
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

    void MinimalEnclosingCircle::findNewSmallestCircle()
    {
        if(this->circle_defining_points_.size() == 3)
        {
            for(Eigen::Vector2f outer_point: this->circle_defining_points_)
            {
                Eigen::Vector2f diff_vector1;
                diff_vector1 << 0, 0;
                Eigen::Vector2f diff_vector2;
                diff_vector2 << 0, 0;
                for(Eigen::Vector2f inner_point: this->circle_defining_points_)
                {
                    if(outer_point != inner_point)
                    {
                        if(diff_vector1[0] == 0 && diff_vector1[1] == 0)
                        {
                            diff_vector1 = inner_point - outer_point;
                        }
                        else
                        {
                            diff_vector2 = inner_point - outer_point;
                        }
                    }
                }
                Eigen::Vector2f diff_vector = diff_vector1 - diff_vector2;
                double angle = atan2(diff_vector[1], diff_vector[0]);

                if(angle > M_PI_2) // If angle is bigger than 90° than it is an obtuse angle
                {
                    std::vector<Eigen::Vector2f>::iterator point_to_delete = std::find(this->circle_defining_points_.begin(),
                                                                                       this->circle_defining_points_.end(),
                                                                                       outer_point);
                    this->circle_defining_points_.erase(point_to_delete);
                    break;
                }
            }
            this->updateCircle();
        }
        else if(this->circle_defining_points_.size() == 4)
        {
            std::vector<Eigen::Vector2f> saved_circle_defining_points = this->circle_defining_points_;
            Eigen::Vector2f new_point = this->circle_defining_points_.back();
            
            std::vector<Eigen::Vector2f> best_circle_defining_points;
            double smallest_circle_radius;

            // Init
            smallest_circle_radius = this->calcDistance(new_point, this->circle_defining_points_[0]);

            // First try forming smallest circle with new point and one of the old points
            for(Eigen::Vector2f point: saved_circle_defining_points)
            {
                if(point != new_point)
                {
                    this->circle_defining_points_.clear();
                    this->circle_defining_points_.push_back(point);
                    this->circle_defining_points_.push_back(new_point);
                    this->updateCircle();
                    if(this->hasCircleAllPointsEnclosed() && this->circle_radius_ < smallest_circle_radius)
                    {
                        best_circle_defining_points = this->circle_defining_points_;
                        smallest_circle_radius = this->circle_radius_;
                    }
                }
            }

            // Then try forming smallest circle with new point and two of the old points
            for(int point_counter = 0; point_counter < 3; point_counter++)
            {
                this->circle_defining_points_.clear();
                this->circle_defining_points_.push_back(saved_circle_defining_points[point_counter]);
                if(point_counter == 2)
                {
                    this->circle_defining_points_.push_back(saved_circle_defining_points[0]);
                }
                else
                {
                    this->circle_defining_points_.push_back(saved_circle_defining_points[point_counter+1]);
                }
                this->circle_defining_points_.push_back(new_point);
                this->updateCircle();

                if(this->hasCircleAllPointsEnclosed() && this->circle_radius_ < smallest_circle_radius)
                {
                    best_circle_defining_points = this->circle_defining_points_;
                    smallest_circle_radius = this->circle_radius_;
                }
            }

            this->circle_defining_points_ = best_circle_defining_points;
            this->updateCircle();
        }
    }

    void MinimalEnclosingCircle::updateCircle()
    {
        if(this->circle_defining_points_.size() == 2)
        {
            this->circle_centre_ = this->calcCentreOfVector(this->enclosed_points_[0], this->enclosed_points_[1]);
            this->circle_radius_ = 0.5 * this->calcDistance(this->enclosed_points_[0], this->enclosed_points_[1]);
        }
        else if(this->circle_defining_points_.size() == 3)
        {
            Eigen::Vector2f vector_centre1 = this->calcCentreOfVector(this->circle_defining_points_[1], this->circle_defining_points_[0]);
            Eigen::Vector2f orthogonal_vector1 = this->calcOrthogonalVector(this->circle_defining_points_[0] - this->circle_defining_points_[1]);
            Eigen::Vector2f vector_centre2 = this->calcCentreOfVector(this->circle_defining_points_[1], this->circle_defining_points_[2]);
            Eigen::Vector2f orthogonal_vector2 = this->calcOrthogonalVector(this->circle_defining_points_[2] - this->circle_defining_points_[1]);

            this->circle_centre_ = this->calcVectorLineIntersectionPoint(vector_centre1, orthogonal_vector1,
                                                                         vector_centre2, orthogonal_vector2);
            this->circle_radius_ = this->calcDistance(this->circle_defining_points_[0], this->circle_centre_);
        }
        else
        {
            std::cout << "MinimalEnclosingCircle: Error during updateCircle method. Number of points define circle: " << std::to_string(this->circle_defining_points_.size());
        }
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

    double MinimalEnclosingCircle::calcDistance(Eigen::Vector2f first_point, Eigen::Vector2f second_point)
    {
        Eigen::Vector2f diff_vector = second_point - first_point;
        return diff_vector.norm();
    }

    bool MinimalEnclosingCircle::hasCircleAllPointsEnclosed()
    {
        for(Eigen::Vector2f point : this->enclosed_points_)
        {
            if(std::find(this->circle_defining_points_.begin(), this->circle_defining_points_.end(), point) == this->circle_defining_points_.end())
            {
                if(this->calcDistance(this->circle_centre_, point) > this->circle_radius_)
                {
                    return false;
                }
            }
        }
        return true;
    }
}