#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField


class ParticleFilter(object):
	def __init__(self):
		self.particles = []

	def add_particle(self, particle):
		self.particles.append(particle)

	def normalize(self):
		""" Normalize all particles using the sum of all of their weights """
		pass

	def integrate_observation(self):
		""" Integrate observations for each of the particles using the observation """
		pass

	def predict(self):
		""" Predict the next position of each of the particles using the odometry of the robot """
		pass

	def get_weighted_values(particles):
		""" Returns a random sample of particles based on how likely they are to be chosen. This means that if a particle is more likely to be in the correct position, it will appear more often in the returned sample """
		pass

	def resample(self):
		""" Update the list of particles using the weighted values. Reset the weights. This should result in a tighter grouping of particles"""
		pass