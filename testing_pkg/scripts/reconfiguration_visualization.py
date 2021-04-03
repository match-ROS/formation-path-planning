#!/usr/bin/env python3

import os

from typing import List

from matplotlib import pyplot as plot
from matplotlib import colors as mcolors

from math import sin, cos, atan2, pi
import numpy as np

def calc_bezier_value(P0: np.array, P1: np.array, P2: np.array, P3: np.array, t:float) -> float:
	value: float = pow(1-t, 3)*P0 + pow(1-t,2)*3*t*P1 + (1-t)*3*pow(t,2)*P2 + pow(t,3)*P3
	return value

if __name__ == '__main__':
	# x
	x_P0: np.array = np.array([0,0])
	x_P1: np.array = np.array([0.33,0])
	x_P2: np.array = np.array([0.66,0])
	x_P3: np.array = np.array([1,0])

	# y
	y_P0: np.array = np.array([0,0])
	y_P1: np.array = np.array([0.33,0])
	y_P2: np.array = np.array([0.66,-1])
	y_P3: np.array = np.array([1,-1])

	counter:float=0.0
	for counter in np.arange(0.0, 1.0, 0.01):
		x_value = calc_bezier_value(x_P0, x_P1, x_P2, x_P3, counter)
		y_value = calc_bezier_value(y_P0, y_P1, y_P2, y_P3, counter)

		plot.figure(0)
		axis: plot.Axes = plot.gca()
		axis.axis("equal")
		plot.title("Beziér-Spline der relativen x-Differenz")
		plot.xlabel("t")
		plot.ylabel("Relative x-Differenz")
		plot.plot(counter, x_value[1], color="black", marker=".", markersize=5)

		plot.figure(1)
		axis: plot.Axes = plot.gca()
		axis.axis("equal")
		plot.title("Beziér-Spline der relativen y-Differenz")
		plot.xlabel("t")
		plot.ylabel("Relative y-Differenz")
		plot.plot(counter, y_value[1], color="black", marker=".", markersize=5)

		plot.figure(2)
		axis: plot.Axes = plot.gca()
		axis.axis("equal")
		plot.title("Verschiebung der Roboterposition im Formationskoordinatensystem")
		plot.xlabel("x-Position im Formationskoordinatensystem")
		plot.ylabel("y-Position im Formationskoordinatensystem")
		plot.plot(x_value[1], y_value[1], color="black", marker=".", markersize=5)

		plot.figure(3)
		axis: plot.Axes = plot.gca()
		axis.axis("equal")
		plot.title("Bewegung des mobilen Roboters im Weltkoordinatensystem")
		plot.xlabel("x-Position im Weltkoordinatensysten")
		plot.ylabel("y-Position im Weltkoordinatensysten")
		plot.plot(-y_value[1], x_value[1]+counter, color="black", marker=".", markersize=5)

	plot.show()
	