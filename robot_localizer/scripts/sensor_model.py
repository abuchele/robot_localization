#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField

import numpy as np
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
		position: a PoseWithCovarianceStamped object representing the
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
		x_pos, y_pos, theta_pos = self.TFHelper.convert_pose_to_xy_and_theta(position)

		direction_radians = np.deg2rad(direction)
		direction_rel = self.TFHelper.angle_normalize(direction_radians + theta_pos)
		x_observation = np.cos(direction_rel) * observation + x_pos
		y_observation = np.sin(direction_rel) * observation + y_pos

		return x_observation, y_observation

	def get_likelihood(self, position, observation, direction):
		"""Calculate the likelihood of a single laser reading coming from a given location.
		
		Parmeters
		---------
		position: a Pose object representing the
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
			return 0.0
		elif distance_to_closest == 1.0:
			return 1.0
		else:
			return norm(0, self.model_noise_rate).pdf(np.abs(distance_to_closest))

	def sample_prediction_new(self, current_odom, destination_odom):
		current_odom_x, current_odom_y, current_odom_theta = TFHelper.convert_pose_to_xy_and_theta(current_odom)
		destination_odom_x, destination_odom_y, destination_odom_theta = TFHelper.convert_pose_to_xy_and_theta(destination_odom)
		delta_odom_x = current_odom_x - destination_odom_x
		delta_odom_y = current_odom_y - destination_odom_y

		# Perform first rotation.
		angle_to_destination = np.atan(delta_odom_y, delta_odom_x)
		rotation_1_angle = TFHelper.angle_diff(current_odom_theta, angle_to_destination)
		rotation_1 = rotation_1_x, rotation_1_y, rotation_1_theta = current_odom_x, current_odom_y, current_odom_theta + rotation_1_angle

		# Perform the translation.
		distance_to_destination = delta_odom_x / np.cos(angle_to_destination)
		translation = translation_x, translation_y, translation_theta = rotation_1_x + distance_to_destination, rotation_1_y, rotation_1_theta

		# Perform second translation.
		rotation_2_angle = TFHelper.angle_diff(translation_theta, destination_odom_theta)
		rotation_2 = rotation_2_x, rotation_2_y, rotation_2_theta = translation_x, translation_y, translation_theta + rotation_2_angle
		return rotation_2_x * np.random.random_sample() * self.odometry_noise_rate, rotation_2_y \
				* np.random.random_sample() * self.odometry_noise_rate, rotation_2_theta * \
				np.random.random_sample() * self.odometry_noise_rate

	def sample_prediction(self, position, delta):
		"""Predict the next position of a particle from a given change in odometry.
		This function adds a small amount of noise to the prediction.

		Parameters
		----------
		position: a PoseWithCovarianceStamped object representing the
			position of a particle.
		delta: a PoseWithCovarianceStamped object representing the change in position of the particle

		Returns
		-------
		The predicted next position of the particle with some random noise added. 
		"""
				# TODO: transform to x y theta tuple to calculate differences and then convert back to odom.

		new_position = Pose()
		new_position.position.x = position.position.x + delta.position.x * np.random.random_sample() * self.odometry_noise_rate
		new_position.position.y = position.position.y + delta.position.y * np.random.random_sample() * self.odometry_noise_rate
		new_position.position.z = position.position.z + delta.position.z * np.random.random_sample() * self.odometry_noise_rate
		
		new_position.orientation.w = position.orientation.w + delta.orientation.w * np.random.random_sample() * self.odometry_noise_rate
		new_position.orientation.x = position.orientation.x + delta.orientation.x * np.random.random_sample() * self.odometry_noise_rate
		new_position.orientation.y = position.orientation.y + delta.orientation.y * np.random.random_sample() * self.odometry_noise_rate
		new_position.orientation.z = position.orientation.z + delta.orientation.z * np.random.random_sample() * self.odometry_noise_rate

		return new_position
