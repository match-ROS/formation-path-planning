#pragma once

namespace fpc_data_classes
{
	/**
     * @brief Contains all settings of the controller that calculates the movement of the robot
     * 
     */
	struct ControllerParams
	{
		//! This calue defines the frequency the controller is called
		float controller_frequency;
		//! Max linear forward velocity of the mobile robot
		float max_vel_x;
		//! Min linear velocity of the mobile robot (if backwards movement are allowed, this is negative)
		float min_vel_x;
		//! Max rotational movement of the mobile robot (no differentiation between left and right movement)
		float max_vel_theta;
		//! Min rotational movement of the mobile robot (no differentiation between left and right movement)
		float min_vel_theta;
	};
}