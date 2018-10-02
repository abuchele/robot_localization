#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion

from helper_functions import TFHelper
from occupancy_field import OccupancyField
from particle_filter import ParticleFilter
from particle import Particle


class ParticleFilterNode(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

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

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.filter = ParticleFilter()
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.position_delta = None
        self.scan = None
        self.odometry = None

    def update_scan(self, msg):
        """Updates the scan to the most recent reading"""
        self.scan = msg.ranges

    def update_position(self, msg):
        """Store change in position since last odometry reading."""
        self.position_delta = Pose()
        self.position_delta.position = Point()
        self.position_delta.orientataion = Quaternion()

        this_pose = msg.pose.pose

        self.position_delta.position.x = this_pose.position.x - self.odometry.position.x
        self.position_delta.position.y = this_pose.position.y - self.odometry.position.y
        self.position_delta.position.z = this_pose.position.z - self.odometry.position.z

        self.position_delta.orientataion.x = this_pose.orientataion.x - self.odometry.orientataion.x
        self.position_delta.orientataion.y = this_pose.orientataion.y - self.odometry.orientataion.y
        self.position_delta.orientataion.z = this_pose.orientataion.z - self.odometry.orientataion.z
        self.position_delta.orientataion.w = this_pose.orientataion.w - self.odometry.orientataion.w

        self.odometry = this_pose

        # TODO: send delta where it's going


    def reinitialize_particles(self, initial_position):
        """Reinitialize particles when a new initial position is given."""
        pass

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # TODO: initialize your particle filter based on the xy_theta tuple

    def publish_particles(self):
        """ Extract pose from each particle and publish them as PoseArray"""
        pose_array = PoseArray()
        for particle in particle_filter.particles:
            pose_array.poses.append(particle.position)
        self.particle_pub.publish(pose_array)

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()

