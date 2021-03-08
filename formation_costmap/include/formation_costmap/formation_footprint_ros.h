#pragma once

#include <geometry_msgs/PolygonStamped.h>

#include <fp_utils/geometry_info/formation_contour.h>
#include <formation_costmap/robot_footprint_ros.h>

namespace formation_costmap
{
    class FormationFootprintRos : public geometry_info::FormationContour
    {
        public:
            /**
             * @brief Default constructor for the FormationContour object
             * 
             */
            FormationFootprintRos();
            /**
             * @brief Construct for Formation Contour object to specify the lead_vector and rotation
             * 
             * @param lead_vector_world_cs Lead vector from world coordinate system to the new coordinate system
             * @param world_to_geometry_cs_rotation Rotation from current to new coordinate system 
             */
            FormationFootprintRos(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation);

			bool addRobotToFormation(std::shared_ptr<geometry_info::RobotContour> robot_to_add) override;

			#pragma region Getter/Setter
			geometry_msgs::PolygonStamped getFormationFootprint();
			#pragma endregion

        private:
			std::map<std::string, bool> robot_position_updates_;

			#pragma region EventCallbacks
			void robotPositionChanged(std::string robot_name) override;
			#pragma endregion

			#pragma region Private Helper Methods
			/**
			 * @brief Checks if all robots in the formation have send an position update
			 * 
			 * @return true All robots have sent an position update
			 * @return false Not all robots have sent an position update
			 */
			bool allPositionUpdatesReceived();

			void resetPositionUpdateTable();
			#pragma endregion
    };
}