#!/usr/bin/python
#
# Plots the results from the 2D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects two optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional
#
# The files have the following format:
#   ID x y yaw_radians

import matplotlib.pyplot as plot
import numpy
import sys
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--initial_poses", dest="optimized_initial_poses",
                  default="/home/cona/PoseGraph-tutorial/Graph_SLAM/poses_optimized_ref.txt", help="The filename that contains the original poses."),
parser.add_option("--observation_poses", dest="optimized_observation_poses",
                  default="/home/cona/PoseGraph-tutorial/Graph_slam/poses_optimized_obs.txt", help="The filename that contains the observation poses.")
(options, args) = parser.parse_args()

# Read the original and optimized poses files.
poses_original = None
if options.optimized_initial_poses != '':
  poses_original = numpy.genfromtxt(options.optimized_initial_poses, usecols = (1, 2))
  
poses_observation = None
if options.optimized_observation_poses != '':
  poses_observation = numpy.genfromtxt(options.optimized_observation_poses, usecols = (1, 2))
  
# Plots the results for the specified poses.
plot.figure()

if poses_original is not None:
  plot.plot(poses_original[:, 0], poses_original[:, 1], '-', label="Original",
            alpha=0.5, color="green")

if poses_observation is not None:
      plot.plot(poses_observation[:, 0], poses_observation[:, 1], '-', label="Observation",
            alpha=0.5, color="red")

if poses_original is not None:
  plot.plot(poses_original[:, 0], poses_original[:, 1], 'o', label="Original",
            alpha=0.5, color="green")

if poses_observation is not None:
      plot.plot(poses_observation[:, 0], poses_observation[:, 1], 'o', label="Observation",
            alpha=0.5, color="red")

plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()
