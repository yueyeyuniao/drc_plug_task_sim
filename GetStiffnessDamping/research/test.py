#!/usr/bin/env python

from numpy import array, pi
from numpy import arange
import time
import openravepy


if __name__ == "__main__":
	env = openravepy.Environment()
	env.Load('env.xml')
	env.SetViewer('qtcoin')
	viewer = env.GetViewer()
	viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
	viewer.SetCamera([
	    [0.,  0., -1., 1.],
	    [1.,  0.,  0., 0.],
	    [0., -1.,  0., 0.],
	    [0.,  0.,  0., 1.]])
	robot = env.GetRobots()[0]
