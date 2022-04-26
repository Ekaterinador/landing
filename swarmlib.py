#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

from math import *
import math
import time
from time import sleep
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
import message_filters
import sys
import numpy as np
import serial
from scipy.integrate import odeint
from tf import TransformListener
from crazyswarm.msg import FullState
from crazyswarm.msg import Position
from  crazyswarm.msg import body_pos
from multiprocessing import Process
import os
from sklearn.neighbors import NearestNeighbors

np.set_printoptions(formatter={'float': '{: 0.2f}'.format})


# Main classes ####################################################################

class Drone:
    def __init__(self, name, shift, cf, leader=False):
        self.name = name
        self.cf = cf
        # self.tf = '/vicon/' + name + '/' + name
        self.tf = '/' + name
        self.leader = leader
        self.tl = TransformListener()
        self.pose = np.array([0, 0, 0])
        self.orient = np.array([0, 0, 0])
        self.sp = np.array([0, 0, 0])
        self.path = Path()
        self.obstacle_update_status = [False, None]
        self.pos_goal = np.array([1.0, 0, 0.7])
        self.shift = shift
        self.file = np.array([0, 0, 0])
        self.file_goal = np.array([0, 0, 0])
        self.yaw = 0.0
        self.cam_coord = np.array([0.0, 0., 0.0])
        self.swarm = []
        self.land_request = 0
        self.shutdown = False


    def position(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.pose = position
        self.file = np.vstack([self.file, self.pose])
        np.save(self.name, self.file)
        return np.array(position)

    def orientation(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.orient = get_angles(np.array(quaternion))
        return get_angles(np.array(quaternion))

    def pub_roboarm(self, cmd_vel):
        name = "/roboarm"
        msg = msg_def_roboarm(cmd_vel)
        pub = rospy.Publisher(name, Position, queue_size=1)
        pub.publish(msg)
        print(name)
        print(msg)

    def publish_sp(self, orient):
        publish_pose(self.sp, orient, self.name + "_sp")

    def publish_position(self):
        publish_pose(self.pose, self.orient, self.name + "_pose")

    def publish_path_sp(self, limit=1000):
        publish_path(self.path, self.sp, self.orient, self.name + "_path", limit)

    def fly(self):
        publish_goal_pos(self.sp, self.yaw, "/" + self.name)
        print('publish ', self.sp)



    def follower(self, pos_goal):
        print('pos goal ', pos_goal)
        print('sel pos ', self.pose)
        dx = pos_goal[0] + self.shift[0] - self.pose[0]
        dy = pos_goal[1] + self.shift[1] - self.pose[1]
        dz = pos_goal[2] + self.shift[2] - self.pose[2]
        target_force = np.array([dx, dy, dz])



        if np.linalg.norm(target_force) < 0.15:
            if self.land_request == 1:
                self.shutdown = 1
                print(self.name, ' shutdown')
            self.land_request = 1
        self.sp = self.pose + limit_speed(target_force, 0.5, 0.5)
        print('sp ', self.sp)

    def cam_data(self, msg):
        pos = msg.transform.translation
        print('pos from camera ', pos)
       # vertical
        x = pos.z
        y = pos.x
        z = pos.y
        # horizontal
        # x = pos.y
        # y = pos.x
        # z = - pos.z
        V = cam_rotation(np.array([x, y, z]), self.yaw)
        print('V ', V)
        self.cam_coord = V
        self.pos_goal = self.pose + V

        self.file_goal = np.vstack([self.file_goal, self.pos_goal])
        np.save(self.name + '_goal', self.file)
        # self.yaw += math.atan(x/y)
        print('GOAL_POS: ', self.pos_goal)


def cam_rotation(V, yaw):
    R_yaw = np.array([[np.cos(yaw), - np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
    print('R ', yaw, R_yaw)

    return R_yaw @ V


class mocap_object:
    def __init__(self, name):
        self.name = name
        # self.tf = '/vicon/' + name + '/' + name
        self.tf = '/' + name
        self.tl = TransformListener()
        self.pose = np.array([0, 0, 0])
        self.orient = np.array([0, 0, 0])

    def position(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.pose = position
        return np.array(position)

    def orientation(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.orient = get_angles(np.array(quaternion))
        self.yaw = self.orient[2]
        return get_angles(np.array(quaternion))

    def publish_position(self):
        publish_pose(self.pose, self.orient, self.name + "_pose")


# Service functions ###############################################################
def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
    name = cf_name + "/cmd_position"
    msg = msg_def_crazyswarm(cf_goal_pos, cf_goal_yaw)
    pub = rospy.Publisher(name, Position, queue_size=1)
    pub.publish(msg)
    print(name)
    print(msg)


def publish_pose(pose, orient, topic_name):
    msg = msg_def_PoseStamped(pose, orient)
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
    pub.publish(msg)


def publish_path(path, pose, orient, topic_name, limit=1000):
    msg = msg_def_PoseStamped(pose, orient)
    path.header = msg.header
    path.poses.append(msg)
    if limit > 0:
        path.poses = path.poses[-limit:]
    pub = rospy.Publisher(topic_name, Path, queue_size=1)
    pub.publish(path)


def get_angles(message):
    quat = (message[0], message[1], message[2], message[3])
    euler = tf.transformations.euler_from_quaternion(quat)
    return euler


def msg_def_crazyswarm(pose, yaw):
    worldFrame = rospy.get_param("~worldFrame", "/world")
    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = pose[0]
    msg.y = pose[1]
    msg.z = pose[2]
    msg.yaw = yaw
    now = rospy.get_time()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    return msg


def msg_def_roboarm(cmd_vel):
    worldFrame = rospy.get_param("~worldFrame", "/world")
    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    print('cmd_vel ', cmd_vel)
    msg.x = cmd_vel[0]
    msg.y = cmd_vel[1]
    msg.z = cmd_vel[2]
    print('x, y, z ', msg.x, msg.y, msg.z)
    msg.yaw = 0
    now = rospy.get_time()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    return msg


def msg_def_PoseStamped(pose, orient):
    worldFrame = "world"
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2])  # 1.57
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    msg.header.seq += 1
    msg.header.stamp = rospy.Time.now()
    return msg


def limit_speed(speed, lim_up, lim_down):
    x = speed[0]
    y = speed[1]
    z = speed[2]
    V = (x ** 2 + y ** 2 + z ** 2) ** 0.5

    if z >= 0 and V >= lim_up:
        return speed * (lim_up / V)
    elif z < 0 and V >= lim_down:
        return speed * (lim_down / V)
    else:
        return speed
