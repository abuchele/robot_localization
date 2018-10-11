#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Vector3

from helper_functions import TFHelper
from occupancy_field import OccupancyField

import numpy as np
import math
from scipy.stats import norm

class SensorModel(object):
	"""This class defines a model for the LIDAR scan.

	Attributes
	----------
	model_noise_rate: accounts for inaccuracies in the laser scan and rounding
		error introduced by the resolution of the OccupancyField used as a map.
	odometry_noise_rate: accounts for error in the odometry reading when
		predicting the next poistion of the particle.
	world_model: an OccupancyField object representing the map the robot is
		localizing to.
	"""
	def __init__(self, model_noise_rate, odometry_noise_rate, world_model, TFHelper):
		self.model_noise_rate = model_noise_rate
		self.odometry_noise_rate = odometry_noise_rate
		self.world_model = world_model	# The odometry field
		self.TFHelper = TFHelper

	def get_coordinate_of_observation(self, position, observation, direction):
		"""Calculate where in the occupancy grid an observation lies
		relative to the particle's position.

		Parameters
		----------
		position: a Vector3 object representing the
			position of a particle.
		observation: a single reading from the LIDAR scan.
		direction: the angle from which the observation came from relative
			to the robot, in degrees.

		Returns
		-------
		A tuple containing the x and y coordinate of the observation in the
			map frame. These coordinates are floating point and are not
			discretized to a single cell on the occupany field.
		"""
		# The position of the particle.
		x_pos = position.x
		y_pos = position.y
		theta_pos = position.z

		direction_radians = np.deg2rad(direction)
		direction_rel = self.TFHelper.angle_normalize(direction_radians + theta_pos)
		x_observation = math.cos(direction_rel) * observation + x_pos
		y_observation = math.sin(direction_rel) * observation + y_pos

		return x_observation, y_observation

	def get_likelihood(self, position, observation, direction):
		"""Calculate the likelihood of a single laser reading coming from a given location.
		
		Parmeters
		---------
		position: a Vector3 object representing the
			position of a particle.
		observation: a single reading from the LIDAR scan.
		direction: the angle from which the observation came from relative
			to the robot, in degrees.

		Returns
		-------
		A floating point value representing the probability of the
			observation coming from the particle at the given position.
		"""
		x_observation, y_observation = self.get_coordinate_of_observation(position, observation, direction)
		distance_to_closest = self.world_model.get_closest_obstacle_distance(x_observation, y_observation)
		if np.isnan(distance_to_closest):
			return float('nan')
		elif distance_to_closest == 1.0:
			return 1.0
		else:
			return norm(0, self.model_noise_rate).pdf(np.abs(distance_to_closest))

	def sample_prediction(self, position, delta):
		"""Predict the next position of a particle from a given change in odometry.
		This function adds a small amount of noise to the prediction.

		Parameters
		----------
		position: a Vector3 object representing the
			position of a particle.
		delta: a Vector3 object representing the change in position of the particle

		Returns
		-------
		new_pose_with_noise: Vector3 object: The predicted next position of the particle with some random noise added. 
		"""
		destination = Vector3()

		# Perform first rotation.
		angle_to_destination = np.arctan2(delta.y, delta.x)
		rotation_1_angle = self.TFHelper.angle_diff(position.z, angle_to_destination)

		rotation_1 = Vector3()

		rotation_1.x = position.x
		rotation_1.y = position.y
		rotation_1.z = self.TFHelper.angle_diff(position.z, rotation_1_angle)

		# Perform the translation.
		distance_to_destination = delta.x / np.cos(angle_to_destination)

		translation = Vector3()

		translation.x = rotation_1.x + distance_to_destination
		translation.y = rotation_1.y
		translation.z = rotation_1.z

		# Perform second rotation.
		rotation_2_angle = self.TFHelper.angle_diff(translation.z, destination.z)

		rotation_2 = Vector3()

		rotation_2.x = translation.x
		rotation_2.y = translation.y
		rotation_2.z = self.TFHelper.angle_diff(translation.z, rotation_2_angle)

		new_pose_with_noise = Vector3()
		
		new_pose_with_noise.x = rotation_2.x + (0.5 - np.random.random_sample()) * self.odometry_noise_rate
		new_pose_with_noise.y = rotation_2.y + (0.5 - np.random.random_sample()) * self.odometry_noise_rate
		new_pose_with_noise.z = rotation_2.z + (0.5 - np.random.random_sample()) * self.odometry_noise_rate

		return new_pose_with_noise

