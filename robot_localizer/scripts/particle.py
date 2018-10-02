#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField


class Particle(object):
	def __init__(self):
		self.pose = Pose()
		self.weight = 0.0
		self.sensor_model = None


	def set_pose(self, pose):
		self.pose = pose

	def set_weight(self, weight):
		self.weight = weight

	def integrate_observation(self):
		pass

	def predict_position(self):
		pass

	def normalize_weight(self):
		pass
		