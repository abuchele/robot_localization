#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField
from particle import Particle
from sensor_model import SensorModel

import numpy as np
from numpy.random import random_sample

class ParticleFilter(object):
	def __init__(self):
		self.particles = []

	def add_particle(self, particle):
		self.particles.append(particle)

	def normalize(self):
		""" Normalize all particles using the sum of all of their weights """
		w_sum = sum([p.weight for p in self.particles])
		[p.normalize_weight(w_sum) for p in self.particles]

	def integrate_observation(self, observation):
		""" Integrate observations for each of the particles using the observation """
		for p in self.particles:
			p.integrate_observation(observation)

	def predict(self, delta):
		""" Predict the next position of each of the particles using the odometry of the robot """
		for p in self.particles:
			p.predict(delta)

	@staticmethod
	def weighted_values(values, probabilities, size):
		""" Return a random sample of size elements from the set values with the specified probabilities
			values: the values to sample from (numpy.ndarray)
			probabilities: the probability of selecting each element in values (numpy.ndarray)
			size: the number of samples
		"""
		bins = np.add.accumulate(probabilities)
		print(probabilities)
		print(bins)
		indices = np.digitize(random_sample(size), bins)
		sample = []
		for ind in indices:
			sample.append(deepcopy(values[ind]))
		return sample

	def resample(self):
		""" Update the list of particles using the weighted values. Reset the weights. This should result in a tighter grouping of particles"""
		self.particles = ParticleFilter.weighted_values(self.particles, [p.weight for p in self.particles], len(self.particles))
		for p in self.particles:
			p.weight = 1./len(self.particles)
