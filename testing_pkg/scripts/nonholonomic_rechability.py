#!/usr/bin/env python3

import os

from typing import List

from matplotlib import pyplot as plot
from matplotlib import colors as mcolors

from math import sin, cos, atan2, pi
import numpy as np
from numpy.core.fromnumeric import size

def gen_point_list(vl_min: float, vl_max: float, vr_min: float, vr_max: float) -> List[List[np.array]]:
	point_list: List[List[np.array]] = list()

	r: float = 0.3
	
	for vl_c in np.arange(vl_min, vr_max, 1.5):
		vl: float = float(vl_c) / 10
		print("vl_c:" + str(vl_c))

		for vr_c in np.arange(vr_min, vr_max, 1.5):
			vr: float = float(vr_c) / 10

			old_x: float = 0.0
			old_y: float = 0.0

			if vl_c==-5.0 or vl_c==10.0 or vr_c==-5.0 or vr_c==10.0:
				one_line: List[np.array] = list()
				for time_c in range(0, 11, 1):
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

					# plot.plot([old_x, x], [old_y, y], linestyle="-", color="black")

					one_line.append(np.array([[x],[y]]))

					# if vl_c==10.0 and vr_c== 10.0:
					# 	print("____________________")
					# 	print(str(x) + str(y))
					# print(str(old_x) + str(old_y))

					old_x = x
					old_y = y

				point_list.append(one_line)

	return point_list


if __name__ == '__main__':
	point_list: List[List[np.array]] = gen_point_list(-5.0, 11.0, -5.0, 11.0)

	# point_list: List[List[np.array]] = gen_point_list(-5.0, 0.0, -5.0, 0.0)
	for counter_outer in range(0, len(point_list)):
		for counter_one_line in range(0, len(point_list[counter_outer])):
			if counter_one_line > 0:
				plot.plot([point_list[counter_outer][counter_one_line-1][0], point_list[counter_outer][counter_one_line][0]], [point_list[counter_outer][counter_one_line-1][1], point_list[counter_outer][counter_one_line][1]], linestyle="-", color="black")
			
			# if counter_outer > 0:
			# 	plot.plot([point_list[counter_outer-1][counter_one_line][0], point_list[counter_outer][counter_one_line][0]], [point_list[counter_outer-1][counter_one_line][1], point_list[counter_outer][counter_one_line][1]], linestyle="-", color="black")
			


	plot.plot(0, 0, marker=".", color="green", markersize=20)

	axis: plot.Axes = plot.gca()  # Get current axis object and set x and y to be equal so a square is a square
	axis.axis("equal")
	plot.show()


