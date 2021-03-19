#!/usr/bin/env python3

import os

from typing import List

from matplotlib import pyplot as plot
from matplotlib import colors as mcolors

from math import sin, cos, atan2, pi
import numpy as np

if __name__ == '__main__':
	point_list: List[np.array] = list()

	# for radius in range(0,10, 1):
	# 	for angle in range(0, 150, 1):
	# 		d_x: float = radius * sin(float(angle)/100)
	# 		d_y: float = radius * (1 - cos(float(angle)/100))

	# 		point_list.append(np.array([d_x, d_y]))

	# point: np.array
	# for point in point_list:
	# 	plot.plot(point[0],
	# 				point[1], color="black", marker=".")

	r: float = 0.3
	
	for vl_c in np.arange(-5.0, 10.0, 1.5):
		vl: float = float(vl_c) / 10
		print("vl_c:" + str(vl_c))

		for vr_c in np.arange(-5.0, 10.0, 1.5):
			vr: float = float(vr_c) / 10

			old_x: float = 0.0
			old_y: float = 0.0

			for time_c in range(0, 10, 1):
				time: float = float(time_c) / 10

				
				angle = ((vl - vr) / ( -2 * r)) * time
				
				x: float
				y: float
				
				if angle == 0.0:
					radius = ((vl + vr) / 2) * time

					x = radius
					y = 0.0
				else:
					radius = ((vl + vr) / (2 * angle)) * time

					x = radius * sin(angle)
					y = radius * (1 - cos(angle))

				# plot.plot(x, y, color="black", marker=".")

				plot.plot([old_x, x], [old_y, y], linestyle="-", color="black")

				# print("____________________")
				# print(str(x) + str(y))
				# print(str(old_x) + str(old_y))

				old_x = x
				old_y = y

	plot.show()


