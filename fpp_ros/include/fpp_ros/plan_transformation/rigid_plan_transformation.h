#pragma once

#include <string>
#include <Eigen/Dense>

namespace plan_transformation
{
	class RigidPlanTransformation
	{
		public:
			#pragma region Constructors
			RigidPlanTransformation() { }
			RigidPlanTransformation(std::string robot_name, Eigen::Vector2f offset);
			#pragma endregion

			#pragma region Getter/Setter
			Eigen::Vector3f getTargetState();
			#pragma endregion

			void updateFormationState(Eigen::Vector3f formation_state);

		protected:
			std::string robot_name_;

			Eigen::Vector3f formation_state_;
			Eigen::Vector3f target_state_;

			Eigen::Vector2f offset_;
	};
}