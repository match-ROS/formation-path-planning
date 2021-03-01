#pragma once

#include <vector>
#include <stdexcept>

namespace fpc_data_classes
{
	struct LyapunovParams
	{
		float kx = 0.0;
		float ky = 0.0;
		float kphi = 0.0;

		LyapunovParams(float kx = 0.0, float ky = 0.0, float kphi = 0.0)
		{
			this->kx = kx;
			this->ky = ky;
			this->kphi = kphi;
		}

		LyapunovParams(std::vector<float> param_list)
		{
			if(param_list.size() != 3)
			{
				throw std::invalid_argument("LyapunovParams: Wrong number of Lyapunov params");
			}
			else
			{
				this->kx = param_list[0];
				this->ky = param_list[1];
				this->kphi = param_list[2];
			}
		}
	};
}