#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField

class Particle(object):
    """ Represents a particle """
    def __init__(self, position, weight, sensor_model):
        # TODO: add documentation to describe what these attributes are.
        self.position = position
        self.weight = weight
        self.sensor_model = sensor_model

    def integrate_observation(self, observation):
        """ integrate an observation """
        # TODO: integrate more than just front/back observations.
        self.weight *= self.sensor_model.get_likelihood(observation.north_laser, self.position, 1)
        self.weight *= self.sensor_model.get_likelihood(observation.south_laser, self.position, -1)

    def predict(self, delta):
        """ predict the next position based on the delta measured using
            the odometry """
        self.position  = self.sensor_model.sample_prediction(self.position+delta)

    def normalize_weight(self, Z):
        """ adjust the particle weight using the specified
            normalization factor (Z) """
        self.weight /= Z
