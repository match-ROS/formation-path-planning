#pragma once

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/geometry_info/edge_info.h>
#include <fpp_ros/geometry_info/geometry_contour.h>

namespace geometry_info
{
    class RobotContour : public geometry_info::GeometryContour
    {
        public:
            /**
             * @brief Default Constuctor for Geometry Contour object.
             * 
             */
            RobotContour() : GeometryContour() { };
            /**
             * @brief Construct for Geometry Contour object to specify the lead_vector and rotation
             * 
             * @param lead_vector_world_cs Lead vector from world coordinate system to the new coordinate system
             * @param world_to_geometry_cs_rotation Rotation from current to new coordinate system 
             */
            RobotContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation, std::string robot_name);

            std::string getRobotName();

        private:
            //! Robot name for identification
            std::string robot_name_;
    };
}