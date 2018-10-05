#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
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
                self.transform_helper = TFHelper()
                self.sensor_model = sensor_model = SensorModel(model_noise_rate=0.05,
                                   odometry_noise_rate=0.1,
                                   world_model=self.occupancy_field,
                                   TFHelper=self.transform_helper)

                self.position_delta = None # Pose, delta from current to previous odometry reading
                self.last_scan = None # list of ranges
                self.odom = None # Pose, current odometry reading

                self.n_particles = 20 # number of particles
                #self.n_particles = 200 # number of particles

                # pose_listener responds to selection of a new approximate robot
                # location (for instance using rviz)
                rospy.Subscriber("map_pose",
                                                 PoseStamped,
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
                self.position_delta = Pose()
                self.position_delta.position = Point()
                self.position_delta.orientation = Quaternion()

                this_pose = msg.pose.pose

                if self.odom is not None:

                        self.position_delta.position.x = this_pose.position.x - self.odom.position.x
                        self.position_delta.position.y = this_pose.position.y - self.odom.position.y
                        self.position_delta.position.z = this_pose.position.z - self.odom.position.z

                        self.position_delta.orientation.x = this_pose.orientation.x - self.odom.orientation.x
                        self.position_delta.orientation.y = this_pose.orientation.y - self.odom.orientation.y
                        self.position_delta.orientation.z = this_pose.orientation.z - self.odom.orientation.z
                        self.position_delta.orientation.w = this_pose.orientation.w - self.odom.orientation.w

                else:
                        self.position_delta = this_pose

                self.odom = this_pose

                self.particle_filter.predict(self.position_delta)


        def reinitialize_particles(self, initial_pose):
                """Reinitialize particles when a new initial pose is given."""
                for i in range(self.n_particles):
                        pose = Pose()
                        pose.position.x = initial_pose.position.x + 2*randn() - 1
                        pose.position.y = initial_pose.position.y + 2*randn() - 1

                        pose.orientation.x = initial_pose.orientation.x + 2*randn() - 1
                        pose.orientation.y = initial_pose.orientation.y + 2*randn() - 1
                        pose.orientation.z = initial_pose.orientation.z + 2*randn() - 1
                        pose.orientation.w = initial_pose.orientation.w + 2*randn() - 1

                        new_particle = Particle(position=pose, weight=1/float(self.n_particles), sensor_model=self.sensor_model)
                        self.particle_filter.add_particle(new_particle)

        def update_initial_pose(self, msg):
                """ Callback function to handle re-initializing the particle filter
                        based on a pose estimate.  These pose estimates could be generated
                        by another ROS Node or could come from the rviz GUI """
                xy_theta = \
                        self.transform_helper.convert_pose_to_xy_and_theta(msg.pose)

                self.reinitialize_particles(msg.pose)


##               # TODO this should be deleted before posting
#try:
#self.transform_helper.fix_map_to_odom_transform(msg.pose, msg.header.stamp)
#except Exception as e:
#print(type(e))
#               # TODO: initialize your particle filter based on the xy_theta tuple

        def publish_particles(self):
                """ Extract pose from each particle and publish them as PoseArray"""
                pose_array = PoseArray()
                pose_array.poses = [p.position for p in self.particle_filter.particles]
                self.particle_pub.publish(pose_array)

        def run(self):
                r = rospy.Rate(5)

                while not(rospy.is_shutdown()):
                        # in the main loop all we do is continuously broadcast the latest
                        # map to odom transform
                        try:
                                self.transform_helper.send_last_map_to_odom_transform()
                                print(len(self.particle_filter.particles), "particles\n")
                                if len(self.particle_filter.particles) > 0:
                                    print([p.w for p in self.particle_filter.particles])
                                    if self.last_scan != None:
                                        self.particle_filter.integrate_observation(self.last_scan)
                                        self.last_scan = None

                                    self.particle_filter.normalize()
                                    self.publish_particles()
                                    self.particle_filter.resample()

                        #except TransformException:
                        except:
                                print("There was a transform exception")

                        r.sleep()

if __name__ == '__main__':
        n = ParticleFilterNode()
        n.run()

