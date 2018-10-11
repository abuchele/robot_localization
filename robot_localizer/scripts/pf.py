#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from helper_functions import TFHelper
from occupancy_field import OccupancyField
from particle_filter import ParticleFilter
from particle import Particle
from sensor_model import SensorModel

from numpy.random import randn, random_sample


class ParticleFilterNode(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        real_robot = False
        # create instances of two helper objects that are provided to you
        # as part of the project
        self.particle_filter = ParticleFilter()
        self.occupancy_field = OccupancyField()
        self.TFHelper = TFHelper()
        self.sensor_model = sensor_model = SensorModel(model_noise_rate=1.9,
                   odometry_noise_rate= 0.1,
                   world_model=self.occupancy_field,
                   TFHelper=self.TFHelper)

        self.position_delta = None # Pose, delta from current to previous odometry reading
        self.last_scan = None # list of ranges
        self.odom = None # Pose, current odometry reading

        self.x_y_spread = 0.2 # Spread constant for x-y initialization of particles
        self.z_spread = 0.2 # Spread constant for z initialization of particles

        self.n_particles = 30 # number of particles
        #self.n_particles = 200 # number of particles

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        rospy.Subscriber("odom", Odometry, self.update_position)
        rospy.Subscriber("stable_scan", LaserScan, self.update_scan)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)


    def update_scan(self, msg):
        """Updates the scan to the most recent reading"""
        self.last_scan = [(i, msg.ranges[i]) for i in range(len(msg.ranges))]

    def update_position(self, msg):
        """Calculate delta in position since last odometry reading, update current odometry reading"""
        self.position_delta = Vector3()

        this_pos = self.TFHelper.convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.odom is not None:
            prev_pos = self.TFHelper.convert_pose_to_xy_and_theta(self.odom)
            self.position_delta.x = this_pos.x - prev_pos.x
            self.position_delta.y = this_pos.y - prev_pos.y
            self.position_delta.z = self.TFHelper.angle_diff(this_pos.z, prev_pos.z)
        else:
            self.position_delta = this_pos

        self.odom = msg.pose.pose

        self.particle_filter.predict(self.position_delta)


    def reinitialize_particles(self, initial_pose):
        """Reinitialize particles when a new initial pose is given."""
        self.particle_filter.particles = []
        for i in range(self.n_particles):
            pos = Vector3()

            initial_pose_trans = self.TFHelper.convert_pose_to_xy_and_theta(initial_pose)

            pos.x = initial_pose_trans.x + (2*randn() - 1) * self.x_y_spread
            pos.y = initial_pose_trans.y + (2*randn() - 1) * self.x_y_spread
            pos.z = initial_pose_trans.z + (2*randn() - 1) * self.z_spread

            new_particle = Particle(position=pos, weight=1/float(self.n_particles), sensor_model=self.sensor_model)
            self.particle_filter.add_particle(new_particle)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        self.reinitialize_particles(msg.pose.pose)

        self.TFHelper.fix_map_to_odom_transform(msg.pose.pose, msg.header.stamp)


    def publish_particles(self):
        """ Extract position from each particle, transform into pose, and publish them as PoseArray"""
        pose_array = PoseArray()
        for p in self.particle_filter.particles:
            pose_array.poses.append(self.TFHelper.convert_vector3_to_pose(p.position))
        pose_array.header.frame_id = "map"
        self.particle_pub.publish(pose_array)


    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.TFHelper.send_last_map_to_odom_transform()
            print(len(self.particle_filter.particles), "particles\n")
            if len(self.particle_filter.particles) > 0:
                if self.last_scan != None:
                    self.particle_filter.integrate_observation(self.last_scan)
                    self.last_scan = None
                    self.particle_filter.normalize()
                    self.publish_particles()
                    self.particle_filter.print_weights()
                    self.particle_filter.resample()
            r.sleep()

if __name__ == '__main__':
    n = ParticleFilterNode()
    n.run()

