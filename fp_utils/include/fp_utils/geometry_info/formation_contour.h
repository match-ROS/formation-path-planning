#pragma once

#include <fp_utils/geometry_info/edge_info.h>
#include <fp_utils/geometry_info/geometry_contour.h>
#include <fp_utils/geometry_info/minimal_enclosing_circle.h>
#include <fp_utils/geometry_info/robot_contour.h>

namespace geometry_info
{
    class FormationContour : public geometry_info::GeometryContour
    {
        public:
            /**
             * @brief Default constructor for the FormationContour object
             * 
             */
            FormationContour();
            /**
             * @brief Construct for Formation Contour object to specify the lead_vector and rotation
             * 
             * @param lead_vector_world_cs Lead vector from world coordinate system to the new coordinate system
             * @param world_to_geometry_cs_rotation Rotation from current to new coordinate system 
             */
            FormationContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation);

            /**
             * @brief Method for executing the gift wrapping algorithm.
             * This uses all points in the corner list and fits a contour on the outer points.
             * After that it deletes all points that are not necessary for this contour
             * 
             */
            void exeGiftWrappingAlg();

            /**
             * @brief Method for adding one robot to the formation contour
             * 
             * @param robot_to_add RobotContour object that represents the robot that should be added to the formation
             */
            virtual bool addRobotToFormation(std::shared_ptr<RobotContour> robot_to_add);

            /**
             * @brief Method for updating the position of one individual robot
             * 
             * @param robot_name Name of the robot of which the position should be changed
             * @param new_robot_pose_world_cs New position where the base_link/cs of the robot should be
             * @param new_rotation_world_to_geometry_cs New rotation from world to geometry cs
             */
            void updateRobotPose(std::string robot_name,
                                 Eigen::Vector2f new_robot_pose_world_cs,
                                 float new_rotation_world_to_geometry_cs);

            /**
             * @brief Method for updating the corner points of the formation
             * Notice that the positions of each robot have to be updated first
             */
            void updateFormationContour();

			/**
             * @brief Move the coordinate system without moving the points of the contour.
             * 
             * @param new_lead_vector_world_cs Lead vector to the new coordinate system
             * @param new_cs_rotation Rotation from the world to the new coordinate system 
             */
            void moveCoordinateSystem(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation) override;

			float calcMinimalEnclosingCircleRadius();

			#pragma region Getter/Setter
			Eigen::Vector2f getRobotPosGeometryCS(std::string robot_name);
			Eigen::Vector2f getRobotPosWorldCS(std::string robot_name);

			std::vector<std::shared_ptr<RobotContour>> getRobotContours();
			std::shared_ptr<RobotContour> getRobotContour(std::string robot_name);
			#pragma endregion

        protected:
            std::vector<std::shared_ptr<RobotContour>> robot_contours_;

			#pragma region EventCallbacks
			virtual void robotPositionChanged(std::string robot_name);
			#pragma endregion

			#pragma region Private Helper Methods
			/**
             * @brief Helper method to find the next point during the gift wrapping algorithm
             * 
             * @param current_wrapping_point_geometry_cs Current point the gift wrapping algorithm has selected in the geometry cs
             * @param corners_to_wrap_geometry_cs All the corners that can be wraped and be the next point in the contour 
             * @param next_wrapping_point_geometry_cs This will be overwritten by the calcNextWrappingPoint method when the next point was found 
             * @param max_radian This will be overwritten by the calcNextWrappingPoint method when the next wrapping point was found
             * @return int Index in the corners_to_wrap_geometry_cs list of the point that the gift wrapping selecte
             */
            int calcNextWrappingPoint(Eigen::Vector2f current_wrapping_point_geometry_cs,
                                      std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs,
                                      Eigen::Vector2f &next_wrapping_point_geometry_cs,
                                      float &max_radian);

            /**
             * @brief Helper method for easy calculation of the atan2 of the vector between a start and end point
             * 
             * @param start_point_geometry_cs Start point of the vector
             * @param end_point_geometry_cs End point of the vector
             * @return float Angle in radian that define the orientation the vector is pointing in
             */
            float calcTan2(Eigen::Vector2f start_point_geometry_cs, Eigen::Vector2f end_point_geometry_cs);
			#pragma endregion
    };
}