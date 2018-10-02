#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField


class SensorModel(object):
	def __init__(self, model_noise_rate, odometry_noise_rate, world_model):
		self.model_noise_rate = model_noise_rate
		self.odometry_noise_rate = odometry_noise_rate
		self.world_model = world_model

	def get_liklihood(self):
		pass

	def sample_prediction(self):
		pass