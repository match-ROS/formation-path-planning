#include <fpp_ros/plan_transformation/rigid_plan_transformation.h>

namespace plan_transformation
{
	#pragma region Constructors
	RigidPlanTransformation::RigidPlanTransformation(std::string robot_name,
													 Eigen::Vector2f offset)
		: robot_name_(robot_name), offset_(offset)
	{ 
		this->target_state_ = Eigen::Vector3f::Zero();
	}
	#pragma endregion

	#pragma region Getter/Setter
	Eigen::Vector3f RigidPlanTransformation::getTargetState()
	{
		return this->target_state_;
	}
	#pragma endregion

	void RigidPlanTransformation::updateFormationState(Eigen::Vector3f new_formation_state)
	{
		// Save old data
		Eigen::Vector3f old_target_state = this->target_state_;
				
		Eigen::Quaternionf new_formation_rotation;
		new_formation_rotation = Eigen::AngleAxisf(new_formation_state[2], Eigen::Vector3f::UnitZ());
		// Eigen::Rotation2Df new_formation_rotation;
		// new_formation_rotation = Eigen::Rotation2Df(new_formation_state[2]);

		Eigen::Vector3f offset_extended;
		offset_extended << this->offset_[0], this->offset_[1], 0.0;

		this->target_state_ = new_formation_state + new_formation_rotation * offset_extended;
		// Maybe change direction later to point to the next point from the target point. 
		// Not in the direction between last and target point.
		this->target_state_[2] = std::atan2(this->target_state_[1] - old_target_state[1],
											this->target_state_[0] - old_target_state[0]);

		// Override new data
		this->formation_state_ = new_formation_state;
	}

	void RigidPlanTransformation::changeRobotOffset(Eigen::Vector2f relative_change)
	{
		this->offset_ = this->offset_ + relative_change;
	}
}