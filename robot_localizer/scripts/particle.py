#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
import numpy as np
import math

from helper_functions import TFHelper
from occupancy_field import OccupancyField

class Particle(object):
	"""Represents a single particle.
	Atrributes
	----------
	position: a Pose representing the position of the particle on the map frame.
	weight: a floating point value representing the likelihood that the
		particle is at the location of the robot.
	sensor_model: a SensorModel representing the LIDAR scan and the
		accompanying noise introduced by the system.
	"""
	def __init__(self, position, weight, sensor_model):
		self.position = position
		self.weight = weight
		self.sensor_model = sensor_model

	def integrate_observation(self, observation):
		"""Integrate an observation and update the weight of the particle.
		
		Parameters
		----------
		observation: a list of tuples of LIDAR readings and the angle at which they came from.
		"""
		for i in range(len(observation)):
			self.weight *= self.sensor_model.get_likelihood(self.position, observation[i][0], observation[i][1])

	def predict(self, delta):
		"""Predict the next position based on the delta measured using
		the odometry
		
		Parameters
		----------
		delta: a Vector3 representing the change position the particle should be moved to.
		"""
		self.position  = self.sensor_model.sample_prediction(self.position, delta)

	def normalize_weight(self, Z):
		"""Adjust the particle weight using the specified
		normalization factor
		
		Parameters
		----------
		Z: an integer normalization factor with which to adjust the particle weight.
		"""
		if Z != 0.0:
			self.weight /= Z
