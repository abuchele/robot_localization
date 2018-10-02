#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField

class Particle(object):
    """ Represents a particle.
    Atrributes
    ----------
    position: a Pose representing the position of the particle on the map frame.
    """
    def __init__(self, position, weight, sensor_model):
        # TODO: add documentation to describe what these attributes are.
        self.position = position
        self.weight = weight
        self.sensor_model = sensor_model

    def integrate_observation(self, observation):
        """ integrate an observation and update the weight of the particle.
        
        Parameters
        ----------
        observation: a list of tuples of LIDAR readings and the angle at which they came from.
        """
        for distance, angle in observation:
            self.weight *= self.sensor_model.get_likelihood(self.position, distance, angle)

    def predict(self, delta):
        """ predict the next position based on the delta measured using
            the odometry """
        self.position  = self.sensor_model.sample_prediction(self.position, delta)

    def normalize_weight(self, Z):
        """ adjust the particle weight using the specified
            normalization factor (Z) """
        self.weight /= Z
