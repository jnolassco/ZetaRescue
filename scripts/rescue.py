#!/usr/bin/env python

"""
    This node wanders around a map randomly searching for ar_tags.

    It navigates to the random goals using the ROS navigation stack.

"""
import sys
import math
import csv
import rospy
import tf
import map_utils
import actionlib
import random

import numpy as np

from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from zeta_rescue.msg import Victim
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

TIME_LIMIT = 120
DISTANCE_TOLERANCE = 1
MARKER_ID = 0
VICTIM_ID = 0

class RescueNode(object):
    def __init__(self, landmarks):
        self.landmarks = self.process_landmarks(landmarks)

        rospy.init_node("rescue")

        self.tf_listener = tf.TransformListener()

        self.point_base = PointStamped()
        self.point_base.header.frame_id = "/base_link"
        self.point_base.header.stamp = rospy.get_rostime()

        rospy.sleep(1.0)

        self.map_msg = None
        self.victim_image = None

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_callback, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        rospy.Subscriber("/report_requested", Empty, self.button_callback)

        self.marker_pub = rospy.Publisher("/victim_marker", Marker, queue_size=10)
        self.image_pub = rospy.Publisher("/victim_image", Image, queue_size=10)
        self.victim_pub = rospy.Publisher("/victim", Victim, queue_size=10)

        self.initial_pose_x = rospy.get_param("initial_pose_x")
        self.initial_pose_y = rospy.get_param("initial_pose_y")
        self.initial_pose_a = rospy.get_param("initial_pose_a")
        self.initial_cov_xx = rospy.get_param("initial_cov_xx")
        self.initial_cov_yy = rospy.get_param("initial_cov_yy")
        self.initial_cov_aa = rospy.get_param("initial_cov_aa")

        self.marker = init_orange_marker()

        self.victims = []

        self.is_currently_handling_victim = False

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.wander()

        # Keep the node up
        while not rospy.is_shutdown():
            rospy.sleep(1)
            continue

        print self.victims

    def  wait_for_current_victim(self):
        rospy.loginfo("Waiting for current victim...")
        while self.is_currently_handling_victim:
            rospy.loginfo("Still waiting for current victim...")
            rospy.sleep(1)


    def process_landmarks(self, landmarks_file):
        landmarks_list = list(csv.reader(open(landmarks_file, "r")))

        landmarks = []

        for row in landmarks_list:
            entry = []
            entry.append(row[0])
            pos = Point(float(row[1]), float(row[2]), 0)
            entry.append(pos)
            landmarks.append(entry)

        return landmarks

    def find_nearest_landmark(self, position):
        min_dist = sys.maxint
        nearest_landmark = self.landmarks[0][0]

        for landmark in self.landmarks:
            dist = distance(position, landmark[1])
            if dist < min_dist:
                min_dist = dist
                nearest_landmark = landmark[0]

        return nearest_landmark


    def wander(self):
        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(.1)

        self.map = map_utils.Map(self.map_msg)

        self.start_time = rospy.get_time()

        run = True
        while not rospy.is_shutdown() and run:
            self.wait_for_current_victim()
            rand_x = random.uniform(-2, 2)
            rand_y = random.uniform(-2, 2)
            while self.map.get_cell(rand_x, rand_y) != 0:
                rand_x = random.uniform(-2, 2)
                rand_y = random.uniform(-2, 2)

            goal = self.goal_message(rand_x, rand_y, "/base_link")
            self.ac.wait_for_server()
            self.ac.send_goal(goal)
            rospy.loginfo("Sending random goal...")

            rospy.sleep(1.0)

            while self.ac.get_state() == GoalStatus.ACTIVE:
                rospy.loginfo("Waiting for current goal to end")
                time = rospy.get_time() - self.start_time
                if time > TIME_LIMIT:
                    rospy.loginfo(time)
                    rospy.loginfo("time limit")
                    run = False
                    break

                rospy.sleep(1)

        rospy.loginfo("Heading Home.")

        goal = self.goal_message(self.initial_pose_x, self.initial_pose_y, "/map")

        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        self.ac.wait_for_result()

        rospy.loginfo("Reached home")

    def goal_message(self, x_target, y_target, frame):
        """ Create a goal message in the base_link coordinate frame"""

        # Create a goal message ...
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = x_target
        goal.target_pose.pose.position.y = y_target
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        return goal

    def victim_goal_message(self, position, quaternion, frame):
        """ Create a goal message in the base_link coordinate frame"""

        # Create a goal message ...
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = position.x
        goal.target_pose.pose.position.y = position.y
        goal.target_pose.pose.position.z = position.z
        goal.target_pose.pose.orientation.x = quaternion.x
        goal.target_pose.pose.orientation.y = quaternion.y
        goal.target_pose.pose.orientation.z = quaternion.z
        goal.target_pose.pose.orientation.w = quaternion.w

        return goal

    def handle_victim(self, ar_pose_msg, ar_tag):
        """
        Attempts to line up with a victim, take a picture and publish a marker
        """
        global VICTIM_ID

        self.is_currently_handling_victim = True

        # Interrupt action lib
        self.ac.cancel_all_goals()

        # Line up with victim
        rospy.loginfo("lining up with victim...")
        ar_tag_frame_id = "/ar_marker_" + str(ar_tag.id)

        quat_list = tf.transformations.quaternion_from_euler(0, 0, 0).tolist()

        victim_position = Point(0, 0, 0.5)
        victim_orientation = Quaternion(quat_list[0], quat_list[1], quat_list[2], quat_list[3])

        victim_goal = self.victim_goal_message(victim_position, victim_orientation, ar_tag_frame_id)

        try:
            rospy.sleep(1)

            # Transform to map frame
            rospy.loginfo("Transforming to map")
            self.tf_listener.waitForTransform(victim_goal.target_pose.header.frame_id,
                "/map",
                victim_goal.target_pose.header.stamp,
                rospy.Duration(1.0))

            map_goal = self.tf_listener.transformPose("/map", victim_goal.target_pose)
            rospy.sleep(1)

            qx = map_goal.pose.orientation.x
            qy = map_goal.pose.orientation.y
            qz = map_goal.pose.orientation.z
            qw = map_goal.pose.orientation.w

            euler_list = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
            quat_list = tf.transformations.quaternion_from_euler(0, 0, euler_list[2] + math.pi/2).tolist()

            map_goal.pose.orientation.x = quat_list[0]
            map_goal.pose.orientation.y = quat_list[1]
            map_goal.pose.orientation.z = quat_list[2]
            map_goal.pose.orientation.w = quat_list[3]

            map_goal.pose.position.z = 0

            map_goal = self.victim_goal_message(map_goal.pose.position, map_goal.pose.orientation, map_goal.header.frame_id)

            print map_goal

            rospy.loginfo("Sending goal")
            self.ac.wait_for_server()
            self.ac.send_goal(map_goal)
            self.ac.wait_for_result()
            self.victims.append(ar_pose_msg)

            VICTIM_ID += 1
            victim = Victim()
            victim.id = VICTIM_ID
            victim.point = ar_pose_msg.position
            victim.image = self.victim_image

            # Publish victim
            self.victim_pub.publish(victim)

            # Publish victim location marker
            rospy.loginfo("Publishing location marker")
            self.marker = init_orange_marker()
            self.marker.pose = ar_pose_msg
            self.marker_pub.publish(self.marker)

            # Spin if no image published yet
            while self.victim_image is None:
                rospy.loginfo("Waiting for image")
                rospy.sleep(0.1)

            # Publish victim image
            rospy.loginfo("Publishing victim image")
            self.image_pub.publish(victim.image)

            # Publish victim
        except tf.Exception as e:
            rospy.loginfo("Failed to line up with victim")
            pass

        self.is_currently_handling_victim = False

    def button_callback(self, button_msg):
        i = 0
        for victim in self.victims:
            rospy.loginfo( "victim %d found at\n%s\nby the landmark: <%s>" % (i, str(victim.position), self.find_nearest_landmark(victim.position)))
            i+=1
        
    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

    def image_callback(self, image_msg):
        """ image_msg will be of type Image """
        self.victim_image = image_msg

    def ar_pose_callback(self, ar_pose_msg):
        """
        Process an ar_tag
        """

        # If a tag is sensed
        if ar_pose_msg.markers and ar_pose_msg.markers[0] is not None:
            ar_tag = ar_pose_msg.markers[0]

            # transform the ar_pose to map frame
            try:
                ar_tag.pose.header.frame_id = ar_tag.header.frame_id

                self.tf_listener.waitForTransform(ar_tag.pose.header.frame_id,
                        "/map",
                        ar_tag.pose.header.stamp,
                        rospy.Duration(1.0))

                ar_tag_map = self.tf_listener.transformPose("/map", ar_tag.pose)
                rospy.sleep(1)

                # If there are no victims so far
                if not self.victims:
                    self.wait_for_current_victim()
                    self.handle_victim(ar_tag_map.pose, ar_tag)
                    pass

                is_new_victim = True

                # Check that it's not the same tag as a previous victim
                for pos in self.victims:
                    ap = pos.position
                    bp = ar_tag_map.pose.position

                    dist = distance(ap, bp)

                    if dist < DISTANCE_TOLERANCE:
                        rospy.loginfo("Duplicate victim detected")
                        is_new_victim = False
                        break

                # new victim (supposedly.. based on distance)
                if is_new_victim:
                    self.wait_for_current_victim()
                    self.handle_victim(ar_tag_map.pose, ar_tag)

            except tf.Exception as e:
                print e


def distance(pos_a, pos_b):
    a = np.array((pos_a.x, pos_a.y, pos_a.z))
    b = np.array((pos_b.x, pos_b.y, pos_b.z))

    # Calculate distance between a and b
    dist = np.linalg.norm(a - b)


def init_orange_marker():
    global MARKER_ID

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = "/map"
    marker.id = MARKER_ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = .7
    marker.scale.y = .7
    marker.scale.z = .7
    marker.color.r = 255
    marker.color.g = 165
    marker.color.b = 0
    marker.color.a = 1

    MARKER_ID += 1

    return marker


if __name__ == "__main__":
    RescueNode(sys.argv[1])

