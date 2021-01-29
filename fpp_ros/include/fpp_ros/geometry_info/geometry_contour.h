#pragma once

//Delete this later
#include "ros/ros.h"

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/geometry_info/edge_info.h>

namespace geometry_info
{
    class GeometryContour
    {
        public:
            /**
             * @brief Default Constuctor for Geometry Contour object.
             * 
             */
            GeometryContour() {};
            /**
             * @brief Construct for Geometry Contour object to specify the lead_vector and rotation
             * 
             * @param lead_vector_world_cs Lead vector from world coordinate system to the new coordinate system
             * @param world_to_geometry_cs_rotation Rotation from current to new coordinate system 
             */
            GeometryContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation);

            /**
             * @brief Add another corner to the contour. 
             * 
             * @param corner_world_cs New corner point provided in the world coordinate system
             */
            void addContourCornerWorldCS(Eigen::Vector2f corner_world_cs);
            /**
             * @brief Add another corner to the contour. 
             * 
             * @param corner_world_cs New corner point provided in the geometry coordinate system
             */
            void addContourCornerGeometryCS(Eigen::Vector2f corner_geometry_cs);
            /**
             * @brief Add list of corner points to the contour
             * 
             * @param corner_list_world_cs List of new corner points that are provided in the world coordinate system
             */
            void addContourCornersWorldCS(std::vector<Eigen::Vector2f> corner_list_world_cs);

            /**
             * @brief Calculate the edges of the contour using the corner points from first to last element
             * 
             */
            void createContourEdges();

            /**
             * @brief Method for calculating the area in the contour. If the area is negative the points are in clockwise order.
             * For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon".
             * 
             * @return float Value of the containing area
             */
            float calcArea();

            /**
             * @brief Method for calculating the absolute area in the contour so the area is always a positive value.
             * For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon".
             * 
             * @return float Value of the containing area
             */
            float calcAbsArea();

            /**
             * @brief Method for calculating the geometric centroid of the contour
             * For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon"
             * 
             * @return Eigen::Vector2f Vector that points to the centroid of the contour in the geometry cs
             */
            Eigen::Vector2f calcCentroidGeometryCS();

            /**
             * @brief Method for calculating the geometric centroid of the contour
             * For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon"
             * 
             * @return Eigen::Vector2f Vector that points to the centroid of the contour in the world cs
             */
            Eigen::Vector2f calcCentroidWorldCS();

            /**
             * @brief Helper method for easy transformation of a point from world to the coordinate system of this object
             * 
             * @param world_cs Point provided in the world coordinate system
             * @return Eigen::Vector2f Point transformed to the geometry coordinate system
             */
            Eigen::Vector2f transformWorldToGeometryCS(Eigen::Vector2f world_cs);
            /**
             * @brief Helper method for easy transformation of a point from the coordinate system of this object to the world coordinate system.
             * 
             * @param geometry_cs Point provided in the geometry coordinate system
             * @return Eigen::Vector2f Point transformed to the world coordinate system
             */
            Eigen::Vector2f transformGeometryToWorldCS(Eigen::Vector2f geometry_cs);

            /**
             * @brief Get a list of all corners of this contour
             * 
             * @return std::vector<Eigen::Vector2f> List of all corner points in world cs
             */
            std::vector<Eigen::Vector2f> getCornerPointsWorldCS();

            /**
             * @brief Move the coordinate system without moving the points of the contour.
             * 
             * @param new_lead_vector_world_cs Lead vector to the new coordinate system
             * @param new_cs_rotation Rotation from the world to the new coordinate system 
             */
            void moveCoordinateSystem(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation);
            /**
             * @brief Move the entire contour. Method leaves corners in the geometry cs the same.
             * Only changes the lead vector and rotation from the world cs.
             * 
             * @param new_lead_vector_world_cs Lead vector to the new position of the coordinate system of the contour
             * @param new_cs_rotation Rotation of the new position of the coordinate system of the contour
             */
            void moveContour(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation);

            /**
             * @brief Operator to compare GeometryContour objects
             */
            friend bool operator==(const GeometryContour &lhs, const GeometryContour &rhs);

        protected:
            /**
             * @brief Create a transformation matrix to transform points from and to the geometry cs of this object
             * 
             * @param lead_vector_world_cs translation from the world to this cs
             * @param rotation Rotation form the world to this cs. Rotation only uses yaw rotation.
             * @return Eigen::Matrix<float, 3, 3> Transformation matrix in the x/y area and yaw rotation.
             */
            Eigen::Matrix<float, 3, 3> createTransformationMatrix(Eigen::Vector2f lead_vector_world_cs, float rotation);

            //! Lead vector from world to this cs
            Eigen::Vector2f lead_vector_world_cs_;
            //! Rotation in radians from the world to this cs
            float world_to_geometry_cs_rotation_;

            //! Transformation matrix from the world to this geometry cs
            Eigen::Matrix<float, 3, 3> tf_world_to_geometry_cs_;
            //! Transformation matrix from the geometry cs to the world cs
            Eigen::Matrix<float, 3, 3> tf_geometry_to_world_cs_;

            //! List that contains all corners of the contour
            std::vector<Eigen::Vector2f> corner_points_geometry_cs_;
            //! List that contains all the edges after calling createContourEdges
            std::vector<geometry_info::EdgeInfo> edge_list_geometry_cs_;
    };
}