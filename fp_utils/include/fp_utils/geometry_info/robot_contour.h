#pragma once

#include <fp_utils/geometry_info/geometry_contour.h>

#include <functional>

namespace geometry_info
{
    class RobotContour : public geometry_info::GeometryContour
    {
        public:
			/**
             * @brief Default Constuctor for Robot Contour object.
             * 
             */
			RobotContour();
			/**
			 * @brief Construct a new named, default Robot Contour object
			 * 
			 * @param robot_name Name of the robot and option to identify the object by
			 */
            RobotContour(std::string robot_name);
            /**
             * @brief Constructor for named Robot Contour object to specify the lead_vector and rotation
             * 
			 * @param robot_name Name of the robot and option to identify the object by
             * @param lead_vector_world_cs Lead vector from world coordinate system to the new coordinate system
             * @param world_to_geometry_cs_rotation Rotation from current to new coordinate system 
             */
			RobotContour(std::string robot_name,
						 Eigen::Vector2f lead_vector_world_cs,
						 float world_to_geometry_cs_rotation);

			void setRobotPoseChangedEventHandler(std::function<void(std::string)> robot_pose_changed_handler);

			void notifyRobotPoseChanged();

			std::string getRobotName();

        protected:
            //! Robot name for identification
            std::string robot_name_;

			//! function pointer to the method of the formation to notify the formation about change in the robot pose
			std::function<void(std::string)> robot_pose_changed_handler_;
    };
}